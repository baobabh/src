#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Authors: Leon Jung, Gilbert, Ashe Kim, Special Thanks : Roger Sacchelli

#필요한 패키지들 설치
import rospy
import time
import numpy as np

from std_msgs.msg import UInt8, Float64, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist #메시지들을 생성하는 부분

from enum import Enum
import math
import tf
from tf import transformations
from ecp_state import ECP_STATE
from ecp_sign_state import ECP_SIGN_STATE

from ecp_preproc.msg import DetectLaneInfo

# 로봇의 주행 제어를 담당
class ControlDriving():
    def __init__(self): #전부 초기화를 담당하는거겠지?

        self.integral = 0.0  #(주영)
        self.dt = 0.1

        #moving internal valiables
        self.theta = 0.0
        self.current_theta = 0.0
        self.last_current_theta = 0.0
        self.lastError = 0.0

        self.sub_moving_state = rospy.Subscriber('/control/moving/state', UInt8, self.get_moving_state, queue_size = 1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size=1)

        self.sub_lane = rospy.Subscriber('/detect/lane', DetectLaneInfo, self.get_lane_info, queue_size = 1)
        self.sub_scan_dis = rospy.Subscriber('/scan', LaserScan, self.get_scan_dis, queue_size=1)
        self.sub_parking_state = rospy.Subscriber('/parking/state', UInt8, self.cbGetParkingState, queue_size = 1)
        self.sub_detect_levelcrossbar = rospy.Subscriber('/detect/levelcrossbar', Bool, self.cbGetLevelCrossBar, queue_size=1)
        self.sub_detect_traffic_signal = rospy.Subscriber('/detect/traffic_signal', UInt8, self.cbGetTrafficSign, queue_size=1)
        
        self.pub_check_parking_ready = rospy.Publisher('/check/parking_ready', UInt8, queue_size = 1)
        self.pub_check_level_cross = rospy.Publisher('/check/levelcross', UInt8, queue_size=1)
        self.pub_detect_lane_side = rospy.Publisher('/detect/lane_side', UInt8, queue_size = 1)
        self.pub_check_traffic = rospy.Publisher('/check/traffic', UInt8, queue_size=1)

        self.pub_check_tunnel = rospy.Publisher('/check/tunnel', UInt8, queue_size=1)

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        #토픽이름 : /cmd_vel    메시지 타입 : 'Twist'    큐 사이즈 : 1

        #moving type enum
        self.TypeOfMoving = Enum('TypeOfMoving', 'idle left right forward backward')
        self.TypeOfState = Enum('TypeOfState', 'idle start stop finish')
          # Type0fMoving은 로봇이 수행할 수 있는 주행 동작을 나타낸다.
            # idle : 주행이 멈춘 상태 / left : 좌회전 / right : 우회전 / forward : 전진 / backward : 후진
          # Type0fState는 로봇의 상태를 나타낸다.
            # idle : 멈춘 상태 / start : 동작시작 / stop : 동작중지 / finish : 동작완료
            # 각 미션을 수행하는데 도움을 주는 듯 하다.

        self.current_pos_x = 0.0
        self.current_pos_y = 0.0
        
        self.desired_center = 160
        self.lane_pos = 0
        
        self.obstacle_trigger = False

        self.parking_state = 0 #0: None, 1:detect parking area, 2: detect end line
        # 0 : 주차가 감지되지 않은 상태
        # 1 : 주차 구역을 감지한 상태(아마 표지판일듯)
        # 2 : 주차 끝 선을 감지한 상태(해당 상태가 되면 주차를 시작 0.4초 회전 후 0.8초 직진)
        #   : (0.1초 정지 후 0.8초 후진 후 동일한 방향으로 0.4초 회전)

        self.scan_dis = [0.0] * 360 #라이더 센서 정보 가져오기 (360도로 스캔)
        self.lastError = 0 #PD제어에서의 에러값을 나타냄
        self.parking_done = False #주차 완료되었는지 여부를 나타냄
        self.parking = False #주차 중이면 True로 변경

        self.level_cross_state = False #크로스바 감지 상태
        self.level_cross_stop = False #크로스바 감지에 의한 상태를 나타냄

        # self.start_ready = True #주행 시작 상태를 나타냄 / True이면 준비가 되었다는 뜻
        self.start_ready = False
        self.traffic = 0 #0: stop, 1: GO

        #DEBUG
        # self.parking_state = 2
        #DEBUG

        #moving params
        self.moving_type = ECP_STATE.NORMAL.value #해당 값을 바꿔서 움직이게 할 수 있다.
        rospy.on_shutdown(self.fnShutDown)
        #loop_rate는 루프의 주기를 설정하는데 사용되는 변수 while not rospy.is_shutdown()을 10hz마다 동작하도록 설정
        time.sleep(5)
        ######################### 해당부분 수정 #########################
        loop_rate = rospy.Rate(10) # 10hz (원래는 10이 아니라 100이었음)



        #self.rotate_90_degrees() #(주영) 처음 시작시 90도 회전하도록 하는 코드 (여기 위치해야함)



 #########################################################################################################################
 ########################################################## 무한루프문 #####################################################        
 #########################################################################################################################

        while not rospy.is_shutdown():
            rospy.loginfo('moving type %d', self.moving_type)
            ###########################################################
            ####################### 미션1 : 신호등 ###################### 우선 start_ready를 False로 해서 테스트중
            ###########################################################
            if self.moving_type == ECP_STATE.NORMAL.value:
                if self.start_ready:
                    msg_check_traffic = UInt8()
                    msg_check_traffic = 1
                    self.pub_check_traffic.publish(msg_check_traffic)
                    if self.traffic == 0: #정지해야 하는 거고
                        self.set_cmd_vel([0,0,0], [0,0,0])
                    else:
                        self.fnNormalDriving() #PD제어를 하는 것
                        self.start_ready = False
                else:
                    self.fnNormalDriving()
            elif self.moving_type == ECP_STATE.INTERSECTION.value:
                self.fnNormalDriving()
            ###########################################################
            ####################### 미션2 : 교차로 #######################
            ###########################################################    
            elif self.moving_type == ECP_STATE.INTERSECTION_LEFT.value:
                msg_lane_side = UInt8()
                msg_lane_side.data = 1
                self.pub_detect_lane_side.publish(msg_lane_side)
                #밑의 if문은 라인을 벗어나면 제어를 하도록 하는 것 같음
                #현재 차선이 왼쪽 차선이 아닐 경우에 우측으로 돌라는 의미
                #lane_pos가 1:왼쪽 차선 / 2:우측 차선
                if self.lane_pos != 1:
                    self.set_cmd_vel([0.08,0,0], [0,0,0.2])
                else:
                    self.fnNormalDriving()
            elif self.moving_type == ECP_STATE.INTERSECTION_RIGHT.value:
                msg_lane_side = UInt8()
                msg_lane_side.data = 2
                self.pub_detect_lane_side.publish(msg_lane_side)
                if self.lane_pos != 2:
                    self.set_cmd_vel([0.08,0,0], [0,0,-0.2])
                else:
                    self.fnNormalDriving()
            ###########################################################
            ####################### 미션3 : 장애물 #######################
            ###########################################################
            elif self.moving_type == ECP_STATE.OBSTACLE.value:
                self.fnConstructionMission()
            ###########################################################
            ####################### 미션4 : 주차 ########################
            ###########################################################
            elif self.moving_type == ECP_STATE.PARKING.value:
                msg_lane_side = UInt8()
                msg_parking_ready = UInt8()
                if self.parking_state == 0: #0 : 주차가 감지되지 않은 상태
                    msg_lane_side.data = 1
                    self.pub_detect_lane_side.publish(msg_lane_side)
                    msg_parking_ready.data = 1
                    self.pub_check_parking_ready.publish(msg_parking_ready)
                    if self.lane_pos != 1:
                        self.set_cmd_vel([0.08,0,0], [0,0,0.7])
                    else:
                        self.fnNormalDriving()
                elif self.parking_state == 1: #1 : 주차가 감지된 상태
                    msg_lane_side.data = 0
                    self.pub_detect_lane_side.publish(msg_lane_side)

                    msg_parking_ready.data = 2
                    self.pub_check_parking_ready.publish(msg_parking_ready)
                    
                    self.fnNormalDriving()
                elif self.parking_state == 2: #2 : 주차 끝 선을 감지한 상태
                    if not self.parking: #초기값이 False라서
                        self.set_cmd_vel([0,0,0], [0,0,0])
                        self.parking = True
                    else:
                        if not self.parking_done:
                            
                            # 레이더를 통해서 좌/우측의 3cm 이하의 데이터 개수 측정
                            lidr_resolution = len(self.scan_dis)
                            degree45 = lidr_resolution//8

                            left_front = degree45
                            left_back = degree45*3

                            right_front = degree45*7
                            right_back = degree45*5

                            left_scan_count = self.fn_cal_scan_count(self.scan_dis[left_front:left_back], 0.3)
                            right_scan_count = self.fn_cal_scan_count(self.scan_dis[right_back:right_front], 0.3)
                            print(self.scan_dis[right_back:right_front])
                            rospy.logwarn("left: %d, right: %d", left_scan_count, right_scan_count)

                            if left_scan_count < right_scan_count:
                            #우측에 장애물이 있다고 판단된 경우
                                # self.fnRotateDriving(90, True)
                                self.fnRotateDriving(0.4, True)
                                self.fnStraightDriving(0.8,True)
                                self.fnStopDriving(0.1)
                                # rospy.logwarn("Parking Done")
                                self.fnStraightDriving(0.8,False)
                                self.fnRotateDriving(0.8, True)
                                # self.fnRotateDriving(90, True)

                                self.parking_done = True
                            else:
                                # self.fnRotateDriving(90, False)

                                self.fnRotateDriving(0.4, False)
                                self.fnStraightDriving(0.8,True)
                                self.fnStopDriving(0.1)
                                # rospy.logwarn("Parking Done")
                                self.fnStraightDriving(0.8,False)
                                self.fnRotateDriving(0.8, False)
                                # self.fnRotateDriving(90, False)
                                self.parking_done = True
                        else:
                            #한쪽 라인밖에 찾지 못한 경우
                            msg_lane_side.data = 1
                            self.pub_detect_lane_side.publish(msg_lane_side)

                            if self.lane_pos != 1:
                                self.set_cmd_vel([0.08,0,0], [0,0,0.2])
                            else:
                                self.fnNormalDriving()
                            
                            msg_parking_ready.data = 0
                            self.pub_check_parking_ready.publish(msg_parking_ready)
            ###########################################################
            ####################### 미션5 : 차단바 #######################
            ###########################################################
            elif self.moving_type == ECP_STATE.LEVELCROSS.value: #크로스바 미션
                msg_check_level_cross = UInt8()
                msg_check_level_cross.data = 1
                self.pub_check_level_cross.publish(msg_check_level_cross)
                # print(self.level_cross_state)
                if not self.level_cross_state:
                    self.fnNormalDriving()
                else:
                    self.set_cmd_vel([0.0,0,0], [0,0,0])
                    self.level_cross_stop = True
                    # rospy.logwarn("Detect Level Cross")
            
            ###########################################################
            ####################### 미션6 : 터널 ########################
            ###########################################################
            elif self.moving_type == ECP_STATE.TUNNEL.value: #터널미션
                #TODO
                msg_check_tunnel = UInt8()
                msg_check_tunnel.data = 1
                self.pub_check_tunnel.publish(msg_check_tunnel)
            loop_rate.sleep()

 #########################################################################################################################
 ########################################################## 무한루프문 #####################################################        
 #########################################################################################################################



    def cbGetTrafficSign(self, msg):
        self.traffic = msg.data

    def cbGetLevelCrossBar(self, msg):
        self.level_cross_state = msg.data

    def cbGetParkingState(self, msg):
        self.parking_state = msg.data

    def fn_cal_angle(self, theta): #필요없는 것 같은데?
        if theta > 2*math.pi:
            theta -= 2*math.pi
        elif theta > math.pi:
            theta = 2*math.pi - theta
        return theta

    def fn_cal_odom_angle(self, theta1, theta2): #필요없는 것 같은데?
        theta = abs(theta2 - theta1)
        return self.fn_cal_angle(theta)
    
    def fn_cal_odom_dis(self, pos1, pos2): #필요없는거 같은데?
        return math.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2)

    # 이 밑에 부분은 주차할 때 필요한 함수들이다.
    def fnStopDriving(self, t):
        self.check_time_pre = time.time()
        while time.time() - self.check_time_pre < t:
            self.set_cmd_vel([0,0,0], [0,0,0])
        
    def fnStraightDriving(self, t, forward):
        self.check_time_pre = time.time()
        if forward:
            while time.time() - self.check_time_pre < t:
                # rospy.loginfo(time.time() - self.check_time_pre)
                self.set_cmd_vel([2.2,0,0], [0,0,0])
            self.set_cmd_vel([0,0,0], [0,0,0])
        else:
            while time.time() - self.check_time_pre < t:
                # rospy.loginfo(time.time() - self.check_time_pre)
                self.set_cmd_vel([-2.2,0,0], [0,0,0])
            self.set_cmd_vel([0,0,0], [0,0,0])

    def fnRotateDriving(self, t, ccw):
        self.check_time_pre = time.time()

        if ccw:
            while time.time() - self.check_time_pre < t:
                self.set_cmd_vel([0,0,0], [0,0,1.8])
            self.set_cmd_vel([0,0,0], [0,0,0])
        else:
            while time.time() - self.check_time_pre < t:
                self.set_cmd_vel([0,0,0], [0,0,-1.8])
            self.set_cmd_vel([0,0,0], [0,0,0])

    # def fnRotateDriving(self, degree, ccw):
    #     self.odom_theta_start = self.current_theta
    #     if ccw:
    #         while self.fn_cal_odom_angle(self.fn_cal_angle(self.odom_theta_start+degree*math.pi/180), self.current_theta)*180/math.pi > 2:
    #             rospy.loginfo(self.fn_cal_odom_angle(self.odom_theta_start+degree*math.pi/180, self.current_theta)*180/math.pi)
    #             self.set_cmd_vel([0,0,0], [0,0,0.6])
    #         self.set_cmd_vel([0,0,0], [0,0,0])
    #     else:
    #         while self.fn_cal_odom_angle(self.fn_cal_angle(self.odom_theta_start-degree*math.pi/180), self.current_theta)*180/math.pi > 2:
    #             rospy.loginfo(self.fn_cal_odom_angle(self.fn_cal_angle(self.odom_theta_start-degree*math.pi/180), self.current_theta)*180/math.pi)
    #             self.set_cmd_vel([0,0,0], [0,0,-0.6])
    #         self.set_cmd_vel([0,0,0], [0,0,0])

    # 노드가 종료될 때 해야 하는 것들 (선속도와 각속도를 0으로 만드는 것)
    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist) 

    def fnNormalDriving(self):
        center = self.desired_center
        error = center - 160

        #P제어 : 응답속도를 빠르게 한다. / 과도한 진동을 만든다.
        #D제어 : 안정성이 높아진다. / 응답속도가 느려진다.
        # Kp = 0.035
        # Kd = 0.007 #원래 주석처리 되어있던 부분

        # Kp = 0.011
        # Kd = 0.01 #초기 

        Kp = 0.018
        Kd = 0.032
        Ki = 0.0001
        

        self.integral += error * self.dt #dt는 시간간격을 의미한다. (line 46,47에 정의)

        angular_z = Kp * error + Ki * self.integral + Kd * (error - self.lastError)
        self.lastError = error

        angular_z = -max(angular_z, -3.0) if angular_z < 0 else -min(angular_z, 3.0)

        self.set_cmd_vel([0.2,0,0], [0,0,angular_z])
        # self.set_cmd_vel([0.15,0,0], [0,0,angular_z])

    #장애물을 피할 때 사용하는 함수
    def fnConstructionMission(self):
        forward_obstacle_count = self.fn_cal_scan_count(self.scan_dis[330:359]+self.scan_dis[0: 30], 0.50)
        if 12 < forward_obstacle_count < 28:
            #라이더를 통해 가져온 전방 -30 ~ 30도 사이의 scan값 중 5cm 미만 데이터 개수
            rospy.loginfo('Forward Obstacle Detecting')
            if self.lane_pos == 1: #left lane
                while self.lane_pos == 1:
                    self.set_cmd_vel([0.1,0,0], [0,0,-0.9])
            elif self.lane_pos == 2: #right lane
                while self.lane_pos == 2:
                    self.set_cmd_vel([0.1,0,0], [0,0,-0.9])
        else:
            self.fnNormalDriving()


    #로봇에게 선속도와 각속도 명령을 전달하는 함수
    def set_cmd_vel(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear[0] #linear : 선속도 벡터
        twist.linear.y = linear[1]
        twist.linear.z = linear[2]
        twist.angular.x = angular[0] #angular : 각속도 벡터
        twist.angular.y = angular[1]
        twist.angular.z = angular[2]
        self.pub_cmd_vel.publish(twist) #twist메시지를 pub_cmd_vel 토픽에 발행

    #장애물 피할 때, 라이더 센서를 말하는 것 같기도 하고?? (정확히 뭔지는 모르겠음)
    def fn_cal_scan_count(self, list_scan_dis, max_dis):
        list_scan_dis = np.array(list_scan_dis)
        return list_scan_dis[list_scan_dis < max_dis].size
        # scan_count = 0
        # for scan_dis in list_scan_dis:
        #     if 0 < scan_dis < max_dis:
        #         scan_count += 1
        # return scan_count
    

    ######### 아래 3개의 함수 전부 노드 내 변수에 저장하는 역할 #########
    def get_lane_info(self, msg):
        # rospy.loginfo(msg)
        self.desired_center = msg.desired_center
        self.lane_pos = msg.lane_pos

    def get_moving_state(self, msg):
        self.moving_type = msg.data

    def get_scan_dis(self, msg):
        self.scan_dis = msg.ranges

        # self.scan_dis = msg.intensities
    ##########################################################

     #정확히 무슨 함수인지 잘 모르겠다. (로봇의 위치를 추출하는 것 같기도 하고?? 그럼 터널미션인건가)    
    def cbOdom(self, odom_msg):
        quaternion = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
        self.current_theta = self.euler_from_quaternion(quaternion)

        if (self.current_theta - self.last_current_theta) < -math.pi:
            self.current_theta = 2. * math.pi + self.current_theta
            self.last_current_theta = math.pi
        elif (self.current_theta - self.last_current_theta) > math.pi:
            self.current_theta = -2. * math.pi + self.current_theta
            self.last_current_theta = -math.pi
        else:
            self.last_current_theta = self.current_theta

        self.current_pos_x = odom_msg.pose.pose.position.x
        self.current_pos_y = odom_msg.pose.pose.position.y

    def euler_from_quaternion(self, quaternion):
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        return theta

    def rotate_90_degrees(self):
        # Twist 메시지를 사용하여 로봇을 90도 회전시키는 명령 발행
        twist = Twist()
        twist.angular.z = 1.57  # 90도 회전에 해당하는 라디안 값 (π/2)
        self.pub_cmd_vel.publish(twist)
        rospy.sleep(1)  # 1초 동안 회전 명령을 유지

        # 회전 후 정지 명령 발행
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

    def main(self):
        rospy.spin()



if __name__ == '__main__':
    rospy.init_node('control_driving')
    node = ControlDriving()
    node.main()
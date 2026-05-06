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

import rospy
import time
import numpy as np
import os

from std_msgs.msg import UInt8, Float64, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from enum import Enum
import math
import tf
from tf import transformations
from ecp_state import ECP_STATE
from ecp_sign_state import ECP_SIGN_STATE

from ecp_preproc.msg import DetectLaneInfov2

class RunningAverage:
    def __init__(self):
        self.total = 0.0
        self.count = 0

    def add_value(self, value):
        self.total += value
        self.count += 1

    def get_average(self):
        if self.count == 0:
            return 0  # Avoid division by zero
        rospy.loginfo("카운트 횟수 : {}".format(self.count))
        return self.total / self.count

class ControlDriving():
    def __init__(self):
        
        self.integral = 0.0  #(주영)
        self.dt = 0.1

        self.Vp = float(os.getenv('VP', 0.2))
        self.Kp = float(os.getenv('KP', 0.015))
        self.Kd = float(os.getenv('KD', 0.00001))
        self.Ki = float(os.getenv('KI', 0.015))

        self.max_error = float("-inf")
        self.min_error = float("inf")

        self.running_avg = RunningAverage()

        #moving internal valiables
        self.theta = 0.0
        self.current_theta = 0.0
        self.last_current_theta = 0.0
        self.LastAngleError = 0.0
        self.LastCenterError = 0.0

        self.sub_moving_state = rospy.Subscriber('/control/moving/state', UInt8, self.get_moving_state, queue_size = 1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size=1)

        self.sub_lane = rospy.Subscriber('/detect/lane', DetectLaneInfov2, self.get_lane_info, queue_size = 1)
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

        self.pub_sign = rospy.Publisher('/detect/traffic', UInt8, queue_size=1)

        #moving type enum
        self.TypeOfMoving = Enum('TypeOfMoving', 'idle left right forward backward')
        self.TypeOfState = Enum('TypeOfState', 'idle start stop finish')
        
        self.current_pos_x = 0.0
        self.current_pos_y = 0.0
        
        self.center_diff = 0
        self.angle_diff = 0
        self.lane_pos = 0
        
        self.obstacle_trigger = False

        self.parking_state = 0 #0: None, 1:detect parking area, 2: detect end line

        self.scan_dis = [0.0] * 360
        self.lastError = 0
        self.parking_done = False
        self.parking = False

        self.level_cross_state = False
        self.level_cross_stop = False

        # self.start_ready = True
        self.start_ready = False
        self.traffic = 0 #0: stop, 1: GO

        ##edited##
        self.no_lines = 0
        self.prev_lane = UInt8()
        self.parking_in = 0
        self.straight = 0
        self.left_obstacle_count = UInt8()
        self.right_obstacle_count = UInt8()
        self.times = 0
        self.angles = 0
        self.flag_levelcross = True
	self.flag_tunnel = True
        ##edited##

        #DEBUG
        # self.parking_state = 2
        #DEBUG

        #moving params
        self.moving_type = ECP_STATE.INTERSECTION.value
        rospy.on_shutdown(self.fnShutDown)
        loop_rate = rospy.Rate(10) # 10hz

        # self.rotate_90_degrees() #(주영) 처음 시작시 90도 회전하도록 하는 코드 (여기 위치해야함)

        flag = True
        while not rospy.is_shutdown():
            rospy.loginfo('moving type %d', self.moving_type)
            if self.moving_type == ECP_STATE.NORMAL.value:
                if self.start_ready:
                    msg_check_traffic = UInt8()
                    msg_check_traffic = 1
                    self.pub_check_traffic.publish(msg_check_traffic)
                    if self.traffic == 0:
                        self.set_cmd_vel([0,0,0], [0,0,0])
                    else:
                        self.fnNormalDriving()
                        self.start_ready = False
                else:
                    self.fnNormalDriving()
            elif self.moving_type == ECP_STATE.INTERSECTION.value:
                # self.set_cmd_vel([0, 0, 0], [0, 0, 0])
                # rospy.loginfo('정지1')
                # rospy.sleep(0.1)
                self.fnNormalDriving()
            elif self.moving_type == ECP_STATE.INTERSECTION_LEFT.value:
                # rospy.loginfo('정지2')
                # self.set_cmd_vel([0, 0, 0], [0, 0, 0])
                rospy.loginfo('하이left : %d',ECP_STATE.INTERSECTION_LEFT.value)
                self.fnIntersection(ECP_STATE.INTERSECTION_LEFT.value)
            elif self.moving_type == ECP_STATE.INTERSECTION_RIGHT.value:
                # rospy.loginfo('정지3')
                # self.set_cmd_vel([0, 0, 0], [0, 0, 0])
                rospy.loginfo('하이right : %d',ECP_STATE.INTERSECTION_LEFT.value)
                self.fnIntersection(ECP_STATE.INTERSECTION_RIGHT.value)
            elif self.moving_type == ECP_STATE.OBSTACLE.value:
                self.fnConstructionMission()
            elif self.moving_type == ECP_STATE.PARKING.value:
                self.fnParkingMission()
            elif self.moving_type == ECP_STATE.LEVELCROSS.value:
                self.fnLevelMission()
            elif self.moving_type == ECP_STATE.TUNNEL.value:
                #TODO
            	self.right_obstacle_count = self.fn_cal_scan_count(self.scan_dis[265:275], 0.3)
		while(self.right_obstacle_count <=8 and self.flag_tunnel):
			if self.lane_pos == 0:
			    self.fnNormalDriving()
			else:
			    self.set_cmd_vel([0.22, 0, 0], [0, 0, 0])
			self.right_obstacle_count = self.fn_cal_scan_count(self.scan_dis[265:275], 0.3)
		self.flag_tunnel = False
                msg_check_tunnel = UInt8()
                msg_check_tunnel.data = 1
                self.pub_check_tunnel.publish(msg_check_tunnel)
            loop_rate.sleep()


    def cbGetTrafficSign(self, msg):
        self.traffic = msg.data

    def cbGetLevelCrossBar(self, msg):
        self.level_cross_state = msg.data

    def cbGetParkingState(self, msg):
        self.parking_state = msg.data

    def fn_cal_angle(self, theta):
        if theta > 2*math.pi:
            theta -= 2*math.pi
        elif theta > math.pi:
            theta = 2*math.pi - theta
        return theta

    def fn_cal_odom_angle(self, theta1, theta2):
        theta = abs(theta2 - theta1)
        return self.fn_cal_angle(theta)
    
    def fn_cal_odom_dis(self, pos1, pos2):
        return math.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2)

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

        # rospy.sleep(1)
        average_value = self.running_avg.get_average()
        home_directory = os.path.expanduser('~')
        file_path = os.path.join(home_directory, 'simulation1.txt')  #example.txt

        rospy.loginfo("최소 : {}".format(self.min_error))
        rospy.loginfo("최대 : {}".format(self.max_error))

        with open(file_path, 'a') as file:
            file.write("Vp: {}\t".format(self.Vp))
            file.write("Kp: {}\t".format(self.Kp))
            file.write("Kd: {}\t".format(self.Kd))
            file.write("Ki: {}\n".format(self.Ki))
            file.write("최소 오차: {}\t".format(self.min_error))
            file.write("최대 오차: {}\n".format(self.max_error))
            file.write("평균 : {}\n".format(average_value))

        print("파일이 성공적으로 저장되었습니다.")

    def fnPID(self, error, lasterror):
        Kp = self.Kp
        Kd = self.Kd
        Ki = self.Ki
        
        self.integral += error * self.dt #dt는 시간간격을 의미한다. (line 46,47에 정의)

        angular_z = Kp * error + Ki * self.integral + Kd * (error - lasterror)
        return angular_z


    def fnNormalDriving(self):
        #현재는 단순히 두 error의 합으로 표현
        #교수님께서 알려주신 방법으로 수정 필요 
        angle_error = self.angle_diff
        center_error = self.center_diff
        angular_z = self.fnPID(angle_error, self.LastAngleError) + self.fnPID(center_error, self.LastCenterError)
        self.LastAngleError = angle_error
        self.LastCenterError = center_error
        angular_z = -max(angular_z, -self.LastAngleError) if angular_z < 0 else -min(angular_z, self.LastAngleError)
        
        #angular_z = abs(angular_z)
        rospy.logwarn(angular_z)
        #if self.LastAngleError<0:
        #    angular_z = -angular_z

        self.set_cmd_vel([self.Vp,0,0], [0,0,angular_z])

    def fnIntersection(self, direction):
        if direction == ECP_STATE.INTERSECTION_RIGHT.value:
            rospy.loginfo('정지111')
            #self.set_cmd_vel([0, 0, 0], [0, 0, 0])
            #if self.lane_pos != 2 and self.lane_pos != 0:
                #self.set_cmd_vel([0, 0, 0], [0, 0, -0.3])
            #else:
            self.fnNormalDriving()
        else:
            rospy.loginfo('정지222')
            #self.set_cmd_vel([0, 0, 0], [0, 0, 0])
            if self.lane_pos != 1 and self.lane_pos != 0:
                self.set_cmd_vel([0, 0, 0], [0, 0, 0.3])
            else:
                self.fnNormalDriving()
        self.right_obstacle_count = self.fn_cal_scan_count(self.scan_dis[260:280], 0.1)
	rospy.loginfo('traffic')
        if self.right_obstacle_count >= 2:
            self.moving_type = ECP_STATE.OBSTACLE.value
            self.pub_sign.publish(self.moving_type)

    def fnConstructionMission(self):
        forward_obstacle_count_right = self.fn_cal_scan_count(self.scan_dis[330:359], 0.4)
        forward_obstacle_count_left = self.fn_cal_scan_count(self.scan_dis[0: 30], 0.4)
        rospy.loginfo("lane_pos: %d", self.lane_pos)
        if 12 < forward_obstacle_count_left + forward_obstacle_count_right < 60:
            rospy.loginfo('Forward Obstacle Detecting')
            if self.lane_pos == 1: #left lane
                if(forward_obstacle_count_left > forward_obstacle_count_right):
                    while self.lane_pos == 1:
                        self.set_cmd_vel([0.0,0,0], [0,0,-0.2])
            elif self.lane_pos == 2: #right lane
                if(forward_obstacle_count_right > forward_obstacle_count_left):
                    while self.lane_pos == 2:
                        self.set_cmd_vel([0.0,0,0], [0,0,0.2])
        elif self.lane_pos == 3:
            self.set_cmd_vel([0.1, 0, 0], [0, 0, 0])
        else:
            self.fnNormalDriving()

    def fnParkingMission(self):
        #sign detecting
        if self.parking_in == 0:
            backward_obstacle_count = self.fn_cal_scan_count(self.scan_dis[170:185], 1)
            self.fnNormalDriving()
            if backward_obstacle_count > 5:
                self.parking_in = 1
        #car detecting
        elif self.parking_in == 1:
            self.left_obstacle_count = self.fn_cal_scan_count(self.scan_dis[80:95], 0.3)
            self.right_obstacle_count = self.fn_cal_scan_count(self.scan_dis[265:280], 0.3)
            if (self.left_obstacle_count + self.right_obstacle_count > 7):
                self.set_cmd_vel([0,0,0],[0,0,0])
                self.parking_in = 2
            if self.lane_pos == 3:
                self.set_cmd_vel([0.05,0,0],[0,0,0])
                self.straight = 1
            else:
                self.fnNormalDriving()
        #turn 90 degree
        elif self.parking_in == 2:
            if self.left_obstacle_count > self.right_obstacle_count:
                self.turn_turtlebot(-1.57, 3)
            else:
                self.turn_turtlebot(1.57, 3)
            self.parking_in = 3
            self.set_cmd_vel([0,0,0], [0,0,0])
        #find end line and parking
        elif self.parking_in == 3:
            msg_parking_ready = UInt8()
            msg_parking_ready.data = 2
            self.pub_check_parking_ready.publish(msg_parking_ready)
            if self.parking_state == 2:
                self.set_cmd_vel([0.05,0,0], [0,0,0])
                rospy.sleep(1)
                self.set_cmd_vel([0,0,0],[0,0,0])
                self.parking_in = 4
                self.set_cmd_vel([-0.05,0,0], [0,0,0])
                rospy.sleep(1)
            else: 
                self.times = self.times + 1
                self.set_cmd_vel([0.1,0,0], [0,0,0])
                rospy.sleep(0.1)
        #back, turn 90 degree
        elif self.parking_in == 4:
            msg_parking_ready = UInt8()
            msg_parking_ready.data = 0
            self.pub_check_parking_ready.publish(msg_parking_ready)
            if self.times>=0 :
                self.set_cmd_vel([-0.1,0,0], [0,0,0])
                self.times = self.times -1
                rospy.sleep(0.1)
            else:
                if self.left_obstacle_count > self.right_obstacle_count:
                    self.turn_turtlebot(-1.57, 3)
                else:
                    self.turn_turtlebot(1.57, 3)
                self.parking_in = 5
        #filtering (not detecting car as sign)
        elif self.parking_in == 5:
            while self.fn_cal_scan_count(self.scan_dis[210:330], 0.3)>=1:
                self.fnNormalDriving()
            self.parking_in = 6
        #detecting sign
        else:
            self.right_obstacle_count = self.fn_cal_scan_count(self.scan_dis[260:280], 0.2)
            if self.right_obstacle_count >= 2:
                self.moving_type = ECP_STATE.LEVELCROSS.value
                self.pub_sign.publish(ECP_SIGN_STATE.STOP.value)
                self.set_cmd_vel([0,0,0],[0,0,0])
            else:
                self.fnNormalDriving() 
        rospy.logwarn("%d",self.parking_in)
    
    def turn_turtlebot(self, desired_angle, time_to_rotate):
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        twist_msg = Twist()

        # Calculate angular velocity
        angular_velocity = desired_angle / time_to_rotate

        # Initialize Twist message
        twist_msg.angular.z = angular_velocity

        # Publish Twist message
        cmd_vel_pub.publish(twist_msg)

        # Sleep for the time required to complete the rotation
        rospy.sleep(time_to_rotate)

        # Stop the robot after completing the turn
        twist_msg.angular.z = 0.0
        cmd_vel_pub.publish(twist_msg)
    
    def fnLevelMission(self):
        msg_check_level_cross = UInt8()
        msg_check_level_cross.data = 1
        self.pub_check_level_cross.publish(msg_check_level_cross)
                
        # print(self.level_cross_state)
        if not self.level_cross_state:
            #self.set_cmd_vel([0.05,0,0], [0,0,0])
            self.fnNormalDriving()
        else:
            self.set_cmd_vel([0.0,0,0], [0,0,0])
            self.level_cross_stop = True
            # rospy.logwarn("Detect Level Cross")

    

        

    #def fnConstructionMission(self):
    #    forward_obstacle_count = self.fn_cal_scan_count(self.scan_dis[-15 :]+self.scan_dis[0: 15], 0.30)
    #    rospy.loginfo('Forward Obstacle Detecting : %d', forward_obstacle_count)
    #    if 12 < forward_obstacle_count < 30 :
    #         # elif self.lane_pos == 2: #right lane
    #             # while self.lane_pos == 2:
    #         if self.obstacle_trigger == False:
    #             self.obstacle_trigger=True
    #             prev_time = time.time()
    #             while time.time() - prev_time < 0.9:
    #                 self.set_cmd_vel([0.2,0,0], [0,0,1.9])
    #             # if self.lane_pos == 1: #left lane
    #                 # while self.lane_pos == 1:
    #             prev_time = time.time()
    #             while time.time() - prev_time < 1.4:
    #                 self.set_cmd_vel([0.2,0,0], [0,0,-1.9])
    #         else:
    #             self.obstacle_trigger=False
    #             prev_time = time.time()
    #             while time.time() - prev_time < 0.9:
    #                 self.set_cmd_vel([0.2,0,0], [0,0,-1.9])
    #             # if self.lane_pos == 1: #left lane
    #                 # while self.lane_pos == 1:
    #             prev_time = time.time()
    #             while time.time() - prev_time < 1.4:
    #                 self.set_cmd_vel([0.2,0,0], [0,0,1.9])
    #     else:
    #         self.fnNormalDriving()


    def set_cmd_vel(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear[0]
        twist.linear.y = linear[1]
        twist.linear.z = linear[2]
        twist.angular.x = angular[0]
        twist.angular.y = angular[1]
        twist.angular.z = angular[2]
        self.pub_cmd_vel.publish(twist) 


    def fn_cal_scan_count(self, list_scan_dis, max_dis):
        list_scan_dis = np.array(list_scan_dis)
        return list_scan_dis[list_scan_dis < max_dis].size
        # scan_count = 0
        # for scan_dis in list_scan_dis:
        #     if 0 < scan_dis < max_dis:
        #         scan_count += 1
        # return scan_count

    def get_lane_info(self, msg):
        rospy.loginfo(msg)
        self.angle_diff = np.radians(msg.angle_diff)
        self.center_diff = msg.center_diff
        self.lane_pos = msg.lane_pos

        # home_directory = os.path.expanduser('~')
        # file_path = os.path.join(home_directory, 'example.txt')

        # with open(file_path, 'a') as file:
        #     file.write("평균 : {}\n".format(average_value))

        # print("파일이 성공적으로 저장되었습니다.")

    def get_moving_state(self, msg):
        self.moving_type = msg.data

    def get_scan_dis(self, msg):
        self.scan_dis = msg.ranges

        # self.scan_dis = msg.intensities
        
        
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
        twist.angular.z = 3.14/2  # 90도 회전에 해당하는 라디안 값 (π/2)
        self.pub_cmd_vel.publish(twist)
        # rospy.sleep(2)
        # twist.angular.z = 3.14/2  # 90도 회전에 해당하는 라디안 값 (π/2)
        # self.pub_cmd_vel.publish(twist)
        rospy.sleep(2)

        # 회전 후 정지 명령 발행
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('control_driving')
    node = ControlDriving()
    node.main()

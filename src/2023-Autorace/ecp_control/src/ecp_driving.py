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


class ControlDriving():
    def __init__(self):
       ###PID### 
        self.integral = 0.0  
        self.lastAngular_z = 0.0
        self.lastError = 0.0
        self.dt = 0.5

        self.Vp = float(os.getenv('VP', 0.11)) 
        self.Kp = float(os.getenv('KP', 0.08))
        self.Ki = float(os.getenv('KI', 0.001))
        self.Kd = float(os.getenv('KD', 0.055))
        self.delta = 0.0
        self.pi = 0.0   


        ###SIGN###
        self.traffic = 0
        self.sub_lane = rospy.Subscriber('/detect/lane', DetectLaneInfov2, self.get_lane_info, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.sub_detect_traffic_signal = rospy.Subscriber('/detect/traffic_signal', UInt8, self.cbGetTrafficSign, queue_size=1)

        rospy.on_shutdown(self.fnShutDown)
        rospy.loginfo("control driving node initialized.")

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
        delta = self.delta
        pi = self.pi
        alpha = 0.2
        beta = 0.8
        error = ((alpha * delta) + (beta * pi))
        
        self.integral += error * self.dt 
        derivative = (error - self.lastError) / self.dt if self.dt > 0 else 0.0

        raw_angular_z = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        angular_z = np.clip(raw_angular_z - self.lastAngular_z, -0.3, 0.3) + self.lastAngular_z
        
        self.lastError = error
        angular_z = max(min(angular_z, 0.6), -0.6)
        self.lastAngular_z = angular_z
        self.set_cmd_vel([self.Vp,0,0], [0,0,-angular_z])

                
       
    
    def set_cmd_vel(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear[0]
        twist.linear.y = linear[1]
        twist.linear.z = linear[2]
        twist.angular.x = angular[0]
        twist.angular.y = angular[1]
        twist.angular.z = angular[2]
        self.pub_cmd_vel.publish(twist)
        rospy.loginfo(angular) 

    def get_lane_info(self, msg):
        self.delta = msg.delta
        self.pi = msg.pi
        rospy.loginfo(msg)
        if self.traffic == 1:
            self.fnNormalDriving()
    
    def cbGetTrafficSign(self, msg):
        self.traffic = msg.data



    def main(self):
        rospy.spin()
        

if __name__ == '__main__':
    rospy.init_node('control_driving')
    node = ControlDriving()
    node.main()

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
from std_msgs.msg import UInt8, Float64

from ecp_state import ECP_STATE
from ecp_sign_state import ECP_SIGN_STATE

class Controller():
    def __init__(self):
        self.sub_sign = rospy.Subscriber('/detect/traffic', UInt8, self.checkSignState, queue_size = 1)

        self.pub_moving_state = rospy.Publisher('/control/moving/state', UInt8, queue_size = 1)

        self.sign_state = ECP_SIGN_STATE.INTERSECTION.value
        self.ecp_state = ECP_STATE.INTERSECTION.value
        self.is_intersection = False
        self.is_parking = False


#    ecp_sign_state.py  ecp_state.py
#    CONSTRUCTION = 0   NORMAL = 0
#    FORBID = 1         INTERSECTION = 1
#    LEFT = 2           INTERSECTION_LEFT = 2
#    PARKING = 3        INTERSECTION_RIGHT = 3
#    RIGHT = 4          OBSTACLE = 4
#    STOP = 5           PARKING = 5
#    TUNNEL = 6         LEVELCROSS = 6
#    INTERSECTION = 7   TUNNEL = 7
#    NONE = 8
#   in autorace 2020.world, The mission is organized in the following order
#   TRAFFIC -> INTERSECTION(LEFT or RIGHT) -> OBSTACLES -> PARKING -> STOP BAR -> TUNNEL
#   ecp_sign_state(ecp_state) 
#   8(0) -> 7(1) -> 2(2) or 4(3) -> 0(4) -> 3(5) -> 2(2) -> 5(6) -> 6(7)

#   new code start
    def checkSignState(self, msg):
        sign_state = ECP_SIGN_STATE(msg.data).value
        ecp_state = UInt8()
	rospy.loginfo("Detect Sign %s %d", ECP_SIGN_STATE(sign_state), sign_state)

        if sign_state == ECP_SIGN_STATE.NONE.value:
            ecp_state = ECP_STATE.NORMAL.value

        elif sign_state == ECP_SIGN_STATE.CONSTRUCTION.value:
            ecp_state = ECP_STATE.OBSTACLE.value
        
        elif sign_state == ECP_SIGN_STATE.FORBID.value:
            ecp_state = ECP_STATE.NORMAL.value

        elif sign_state == ECP_SIGN_STATE.INTERSECTION.value:
            ecp_state = ECP_STATE.INTERSECTION.value

        elif sign_state == ECP_SIGN_STATE.LEFT.value:
            ecp_state = ECP_STATE.INTERSECTION_LEFT.value

        elif sign_state == ECP_SIGN_STATE.RIGHT.value:
            ecp_state = ECP_STATE.INTERSECTION_RIGHT.value

        elif sign_state == ECP_SIGN_STATE.PARKING.value:
            ecp_state = ECP_STATE.PARKING.value

        elif sign_state == ECP_SIGN_STATE.STOP.value:
            ecp_state = ECP_STATE.LEVELCROSS.value            

        elif sign_state == ECP_SIGN_STATE.TUNNEL.value:
            ecp_state = ECP_STATE.TUNNEL.value

        if (self.ecp_state + 1 == ecp_state) or (self.ecp_state == ECP_STATE.INTERSECTION.value and sign_state == ECP_SIGN_STATE.RIGHT.value) or (self.ecp_state == ECP_SIGN_STATE.LEFT.value and sign_state == ECP_SIGN_STATE.CONSTRUCTION.value):
            self.sign_state = sign_state
            self.changeSignState()
        

#   new code end
    def changeSignState(self):
        rospy.logwarn("Detect %s Sign", ECP_SIGN_STATE(self.sign_state))
        msg_moving_state = UInt8()

        if self.sign_state == ECP_SIGN_STATE.NONE.value:
            self.ecp_state = ECP_STATE.NORMAL.value

        elif self.sign_state == ECP_SIGN_STATE.CONSTRUCTION.value:
            self.ecp_state = ECP_STATE.OBSTACLE.value
            self.is_intersection = False
        
        elif self.sign_state == ECP_SIGN_STATE.FORBID.value:
            self.ecp_state = ECP_STATE.NORMAL.value

        elif self.sign_state == ECP_SIGN_STATE.INTERSECTION.value:
            self.ecp_state = ECP_STATE.INTERSECTION.value
            self.is_intersection = True

        elif self.sign_state == ECP_SIGN_STATE.LEFT.value:
            if self.is_intersection:
                self.ecp_state = ECP_STATE.INTERSECTION_LEFT.value
            elif self.is_parking:
                pass

        elif self.sign_state == ECP_SIGN_STATE.RIGHT.value:
            self.ecp_state = ECP_STATE.INTERSECTION_RIGHT.value

        elif self.sign_state == ECP_SIGN_STATE.PARKING.value:
            self.ecp_state = ECP_STATE.PARKING.value
            self.is_parking = True

        elif self.sign_state == ECP_SIGN_STATE.STOP.value:
            self.ecp_state = ECP_STATE.LEVELCROSS.value
            self.is_parking = False
            

        elif self.sign_state == ECP_SIGN_STATE.TUNNEL.value:
            self.ecp_state = ECP_STATE.TUNNEL.value

        msg_moving_state.data = self.ecp_state
        self.pub_moving_state.publish(msg_moving_state)




    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('contorl')
    node = Controller()
    node.main()

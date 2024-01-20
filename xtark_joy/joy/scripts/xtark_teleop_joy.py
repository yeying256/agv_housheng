#!/usr/bin/env python

#################################################################################
# Copyright 2022 XTARK CO., LTD. All rights reserved.
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
#################################################################################
# Authors: Eric #
# 手柄远程遥控机器人节点
#################################################################################


import importlib

import rospy
from sensor_msgs.msg import Joy
import time
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String as StringMsg
from agv_msg.msg import Button
from agv_msg.srv import grab_agv

# X:急停(2)、Y:解锁(3)、A:下一步(0)、B:回初始点(1)
func_list = ["next_step", "go_back", "stop", "unlock", "lift", "grab_reset", "task start", "grab_open"]


class JoyTeleop:
    def __init__(self):
        self.x_speed_scale = rospy.get_param("~x_speed_scale")
        self.y_speed_scale = rospy.get_param("~y_speed_scale")
        self.w_speed_scale = rospy.get_param("~w_speed_scale")
        self.velocity = Twist()
        self.rate = rospy.Rate(20)
        self.active = 0
        self.enable = 0
        self.button = Button()
        self.cmdVelPublisher = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        self.cmdButtonPublisher = rospy.Publisher("/push_button", Button, queue_size=1)
        self.joySubscriber = rospy.Subscriber("joy", Joy, self.buttonCallback)
        self.loop()

    def buttonCallback(self, joy_data):
        if joy_data.buttons[4] == 1:
            self.velocity.linear.x = self.x_speed_scale * joy_data.axes[4]
            self.velocity.linear.y = self.y_speed_scale * joy_data.axes[3]
            self.velocity.angular.z = self.w_speed_scale * joy_data.axes[0]
            self.active = 1
        else:
            self.velocity.linear = Vector3(0.0, 0.0, 0.0)
            self.velocity.angular = Vector3(0.0, 0.0, 0.0)
            self.active = 0
            self.cmdVelPublisher.publish(self.velocity)

            # 四种状态
            self.button.button_type = []
            self.button.button_func = []
            
                    
            # 夹爪抬升
            if joy_data.buttons[5] == 1:
                for i in range(4):
                    if joy_data.buttons[i] == 1:
                        if i == 2:
                            self.button.button_type.append(4)
                            self.button.button_func.append(func_list[4])
                        if i == 3:
                            self.button.button_type.append(5)
                            self.button.button_func.append(func_list[5])
                        if i == 0:
                            self.button.button_type.append(6)
                            self.button.button_func.append(func_list[6])
                        if i == 1:
                            self.button.button_type.append(7)
                            self.button.button_func.append(func_list[7])
            else:
                for i in range(4):
                    if joy_data.buttons[i] == 1:
                        if i == 2:
                            self.enable = 0
                        if i == 3:
                            self.enable = 1
                        self.button.button_type.append(i)
                        self.button.button_func.append(func_list[i])

            if len(self.button.button_type):
                self.cmdButtonPublisher.publish(self.button)

    def loop(self):
        while not rospy.is_shutdown():
            if self.active == 1 & self.enable == 1:
                self.cmdVelPublisher.publish(self.velocity)
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("teleop_joy")
    joy = JoyTeleop()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("exception")

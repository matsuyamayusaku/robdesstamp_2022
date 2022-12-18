#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2018 RT Corporation
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

import rospy
import sys
import moveit_commander
import actionlib
import geometry_msgs.msg
import rosnode
import random
from tf import transformations
from tf.transformations import quaternion_from_euler
import math
from geometry_msgs.msg import Point
from robotdesign3_2021_1.msg import CustomArray
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal
)


class Home(object):

    def __init__(self):
        rospy.init_node("motion")
        rospy.Subscriber('/hand_topic', CustomArray, self.callback, queue_size=1)
        self._client = actionlib.SimpleActionClient("/crane_x7/gripper_controller/gripper_cmd",GripperCommandAction)
        self._goal = GripperCommandGoal()
        self.robot = moveit_commander.RobotCommander()
        self.array_points = CustomArray()

        self._client.wait_for_server(rospy.Duration(10.0))
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("txiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)
        self.clear()
        
        self.hand1_x = 0
        self.hand1_y = 0

    def command(self, position, effort):
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        self._client.send_goal(self._goal,feedback_cb=self.feedback)
    
    def callback(self, msg):
    
        self.hand1_x = msg.points[0].x
        self.hand1_y = msg.points[0].y
        print(self.hand1_x, self.hand1_y)

    def feedback(self,msg):
        print("feedback callback")
        print(msg)

    def stop(self):
        self._client.cancel_goal()
        
    def wait(self, timeout=0.1 ):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result()

    def clear(self):
        self._goal = GripperCommandGoal()


    def conversion(self):
        # カメラ座標を持ってくる
        self.cam_x = self.hand1_x
        self.cam_y = self.hand1_y
        inversion = -1 #カメラが逆さに付いているので
        ratio_cm = 0.05 #[cm] あるカメラ座標の値のときのアーム座標のずれ
        self.cam_x = (self.cam_x + ratio_cm) * inversion
        self.cam_y = (self.cam_y + ratio_cm) * inversion
        print(self.cam_y)
        #print('\n\033[33m' + 'x:' + self.cam_x + 'y:' + self.cam_y + '\033[0m\n')


    def motion(self):
        gc = Home()
        arm = moveit_commander.MoveGroupCommander("arm")
        arm.set_max_velocity_scaling_factor(0.12)
        arm.set_max_acceleration_scaling_factor(0.5)
        gripper = moveit_commander.MoveGroupCommander("gripper")

        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(1.0)
        rospy.sleep(1.0)

        print("Group names:")
        print(self.robot.get_group_names())

        print("Current state:")
        print(self.robot.get_current_state())

        # アーム初期ポーズを表示
        arm_initial_pose = arm.get_current_pose().pose
        print("Arm initial pose:")
        print(arm_initial_pose)

        # 何かを掴んでいた時のためにハンドを開く
        gripper.set_joint_value_target([0.9, 0.9])
        gripper.go()

        # SRDFに定義されている"home"の姿勢にする
        arm.set_named_target("home")
        arm.go()
        gripper.set_joint_value_target([0.7, 0.7])
        gripper.go()

        # 顔を認識するふり（ハンドを動かす）
        for gupa in range(2):
            gripper.set_joint_value_target([0.9, 0.9])
            gripper.go()
            rospy.sleep(0.5)
            gripper.set_joint_value_target([0.3, 0.3])
            gripper.go()
            rospy.sleep(0.5)

        #確率で左右どちらかを決める
        ran = random.randint(1, 2)
        if ran == 1:
            pos_y = 0.2
        else:
            pos_y = -0.2

        # 掴む準備をする
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.2
        target_pose.position.y = pos_y
        target_pose.position.z = 0.3
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

        # ハンドを開く
        gripper.set_joint_value_target([0.7, 0.7])
        gripper.go()

        # 掴みに行く
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.2
        target_pose.position.y = pos_y
        target_pose.position.z = 0.11
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

        # ハンドを閉じる
        gripper.set_joint_value_target([0.2, 0.2])
        gripper.go()

        # 持ち上げる
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.2
        target_pose.position.y = pos_y
        target_pose.position.z = 0.3
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        print('\n\033[33mI got a stamp!\033[0m\n')
        arm.go()							# 実行

        # 移動する
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.3
        target_pose.position.y = 0.0
        target_pose.position.z = 0.3
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行
        print("\n\033[33mI'll stamp!!\033[0m\n")
        rospy.sleep(2.0)

        #self.conversion()


    
        # 下ろす
        hand1_x_buff = 0.26 + self.hand1_y - 0.045
        hand1_y_buff = (self.hand1_x + 0.05)
        target_pose = geometry_msgs.msg.Pose()
        target_joint_values = arm.get_current_joint_values()
        target_pose.position.x = hand1_x_buff
        target_pose.position.y = hand1_y_buff
        print(target_pose.position.x, target_pose.position.y)
        #target_pose = geometry_msgs.msg.Pose()
        #target_pose.position.x = 0.3 + self.cam_x - 0.01
        #target_pose.position.y = 0.0 + self.cam_y
        target_pose.position.z = 0.12
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行
        print('\n\033[33mpon\033[0m\n')#####
        rospy.sleep(1.0)

        # 少しだけハンドを持ち上げる
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.3
        target_pose.position.y = 0.0
        target_pose.position.z = 0.2
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

        # SRDFに定義されている"home"の姿勢にする
        # どんなスタンプを押したのか見せつける
        arm.set_named_target("home")
        arm.go()
        print('\n\033[33mTa-da!!!!\033[0m\n')#######
        rospy.sleep(2.0)


        # スタンプを元の位置に戻す
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.15
        target_pose.position.y = pos_y
        target_pose.position.z = 0.10
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

        # ハンドを開く
        gripper.set_joint_value_target([0.7, 0.7])
        gripper.go()

        # 上げる
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.15
        target_pose.position.y = pos_y
        target_pose.position.z = 0.3

        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()							# 実行


        # SRDFに定義されている"home"の姿勢にする
        arm.set_named_target("home")
        arm.go()



        print("done")

    def run(self):
        try:
            while not rospy.is_shutdown():
                #enterが押されるまで待機
                try:
                    input('\n\033[33m__STANDBY__\npress"enter"\033[0m\n')
                    print('START')
                except:
                    print('START')
                
                self.motion()
        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':
    while not rospy.is_shutdown():
        Home().run()
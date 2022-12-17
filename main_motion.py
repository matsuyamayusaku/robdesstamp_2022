#! /usr/bin/env python
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
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32
import message_filters
import random

class Home(object):

    def conversion(self):
        try:
            # カメラ座標を持ってくる
            self.cam_x = message_filters.Subscriber("hand_topic_x", Float32)
            self.cam_y = message_filters.Subscriber("hand_topic_y", Float32)
            inversion = -1 #カメラが逆さに付いているので
            ratio_cm = 0.05 #[cm] あるカメラ座標の値のときのアーム座標のずれ
            self.cam_x = (cam_x + ratio_cm) * inversion
            self.cam_y = (cam_y + ratio_cm) * inversion
            print('x:' + cam_x + 'y:' + cam_y)
        except:
            self.cam_x = 0
            self.cam_y = 0
            print('no camera coordinates')

    def motion(self):
        rospy.init_node("motion")
        self.robot = moveit_commander.RobotCommander()
        arm = moveit_commander.MoveGroupCommander("arm")
        arm.set_max_velocity_scaling_factor(0.12)
        arm.set_max_acceleration_scaling_factor(0.5)
        gripper = moveit_commander.MoveGroupCommander("gripper")

        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(1.0)
        rospy.sleep(1.0)

        print("Group names:")
        print(robot.get_group_names())

        print("Current state:")
        print(robot.get_current_state())

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
        for gupa in range(3):
            gripper.set_joint_value_target([0.9, 0.9])
            gripper.go()
            rospy.sleep(0.7)
            gripper.set_joint_value_target([0.3, 0.3])
            gripper.go()
            rospy.sleep(0.7)




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
        target_pose.position.z = 0.10
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

        # ハンドを閉じる
        gripper.set_joint_value_target([0.4, 0.4])
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
        print('I got a stamp!')
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
        print("I'll stamp!!")

        self.conversion()

        # 下ろす
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.43 + self.conversion.cam_x
        target_pose.position.y = 0.0 + self.conversion.cam_y
        target_pose.position.z = 0.10
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行
        print('pon')
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
        print('Ta-da!!!!')
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

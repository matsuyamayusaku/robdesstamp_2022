#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
import actionlib
import geometry_msgs.msg
import rosnode
from tf import transformations
from tf.transformations import quaternion_from_euler
import random
from geometry_msgs.msg import Point
from RobdeStamp_2022.msg import CustomArray
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal
)

class Home(object):
    def __init__(self):
        rospy.init_node("Hand_MediaPipe")
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
        self.hand1_z = 0

        self.hand2_x = 0
        self.hand2_y = 0
        self.hand2_z = 0
        
        self.hand3_x = 0
        self.hand3_y = 0
        self.hand3_z = 0

        self.hand4_x = 0
        self.hand4_y = 0
        self.hand4_z = 0

    def command(self, position, effort):
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        self._client.send_goal(self._goal,feedback_cb=self.feedback)

    def callback(self, msg):
        
        self.hand1_x = msg.points[0].x
        self.hand1_y = msg.points[0].y
        self.hand1_z = msg.points[0].z

        self.hand2_x = msg.points[1].x
        self.hand2_y = msg.points[1].y
        self.hand2_z = msg.points[1].z

        self.hand3_x = msg.points[2].x
        self.hand3_y = msg.points[2].y
        self.hand3_z = msg.points[2].z

        self.hand4_x = msg.points[3].x
        self.hand4_y = msg.points[3].y
        self.hand4_z = msg.points[3].z


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


    
    #ここからペースト
    def main():
        rospy.init_node("motion")
        robot = moveit_commander.RobotCommander()
        arm = moveit_commander.MoveGroupCommander("arm")
        arm.set_max_velocity_scaling_factor(0.1)
        arm.set_max_acceleration_scaling_factor(1.0)
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
            gripper.set_joint_value_target([0.3, 0.3])
            gripper.go()




        #確率で左右どちらかを決める
        ran = random.randint(1, 2)
        if ran == 1:
            pos_y = 0.15
        else:
            pos_y = -0.15





        # 掴む準備をする
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
        arm.go()  # 実行

        # ハンドを開く
        gripper.set_joint_value_target([0.7, 0.7])
        gripper.go()

        # 掴みに行く
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

        # ハンドを閉じる
        gripper.set_joint_value_target([0.4, 0.4])
        gripper.go()

        # 持ち上げる
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

        # 下ろす
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.3
        target_pose.position.y = 0.0
        target_pose.position.z = 0.10
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行


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
        arm.set_named_target("home")
        arm.go()

        print("done")


    if __name__ == '__main__':

        try:
            if not rospy.is_shutdown():
                main()
        except rospy.ROSInterruptException:
            pass

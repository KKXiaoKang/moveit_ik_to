#!/usr/bin/env python3
"""Kuavo机器人基础手臂位置控制案例展示

这个案例演示了如何使用Kuavo机器人SDK控制机器人的手臂进行位置控制。

功能描述：
- 初始化ROS节点
- 初始化Kuavo机器人实例
- 将机器人手臂控制模式设置为位置规划模式
- 发布手臂关节数据，控制手臂运动到指定位置
- 等待一段时间
- 控制手臂移动到其他位置
- 手臂归中
- 关闭手臂控制
"""

import rospy
import time
from kuavoRobotSDK import kuavo
from utils import rad_to_angle

angle_joint_end0 = rad_to_angle([-0.9636, -0.2836, 0.2506, -0.2406, 0.0704, -0.0733, 0.1217, 0, 0, 0, 0, 0, 0, 0])
angle_joint_end1 = rad_to_angle([-0.9623, -0.2593, -0.0668, -0.265,  -0.0067, -0.0147, -0.0367, 0, 0, 0, 0, 0, 0, 0])
angle_joint_end2 = rad_to_angle([-0.9048, -0.2441, -0.1065, -0.3311, -0.6778, -0.1039, -0.1589, 0, 0, 0, 0, 0, 0, 0])

print(angle_joint_end2)

if __name__ == "__main__":
    # 初始化节点
    rospy.init_node('demo_test') 

    # 初始化机器人
    robot_instance = kuavo("3_7_kuavo")

    # 控制进入到手臂规划模式
    robot_instance.set_robot_arm_ctl_mode(True)

    # 发布关节数据控制
    time.sleep(1) #
    joint_positions = angle_joint_end2
    robot_instance.set_arm_traj_position(joint_positions)


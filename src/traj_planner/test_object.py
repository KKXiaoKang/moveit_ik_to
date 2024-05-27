#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import sys
import moveit_commander
import rospy
import geometry_msgs.msg

from planner import Planner  # 确认 planner.py 中 Planner 类已正确实现
from logger import Logger  # 确认 logger.py 中 Logger 类已正确实现
from publisher import Publisher  # 确认 publisher.py 中 Publisher 类已正确实现

from vision_msgs.msg import Detection2DArray, Detection2D

def detection_callback(msg):
    if not msg.detections:
        rospy.loginfo("No detections received.")
        return

    # 假设你只取第一个检测到的物体进行规划
    detection = msg.detections[0]

    # 提取目标的笛卡尔坐标
    target_pose = detection.results[0].pose

    # 转换为MoveIt可用的格式
    target_pose_stamped = geometry_msgs.msg.PoseStamped()
    target_pose_stamped.header.frame_id = "base_link"
    target_pose_stamped.pose.position.x = target_pose.pose.position.x
    target_pose_stamped.pose.position.y = target_pose.pose.position.y
    target_pose_stamped.pose.position.z = target_pose.pose.position.z
    target_pose_stamped.pose.orientation = target_pose.pose.orientation  # 使用检测到的朝向
    print(" target_pose_stamped : ", target_pose_stamped)

    # 获取当前状态
    now_robot_state = planner.get_current_joints_values()
    # print("now_robot_state : ", now_robot_state)

    # 设置当前状态作为起始状态
    planner.set_start_state(now_robot_state)

    # 规划路径
    traj = planner.plan_to_target_pose(target_pose_stamped)

    if traj is None:
        rospy.logerr("规划失败，未生成有效轨迹")
        return

    # 记录轨迹
    logger.dump_traj(traj, file_name="detection_traj")

if __name__ == "__main__":
    rospy.init_node("test_node", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    planner = Planner()
    logger = Logger()
    publisher = Publisher()

    publisher.start_auto_publish()
    logger.make_traj_dir()

    # 订阅 /object_yolo_tf2_torso_result 话题
    detection_subscriber = rospy.Subscriber("/object_yolo_tf2_torso_result", Detection2DArray, detection_callback)

    rospy.spin()  # 保持节点运行

    # 结束后停止发布
    publisher.stop_auto_publish()

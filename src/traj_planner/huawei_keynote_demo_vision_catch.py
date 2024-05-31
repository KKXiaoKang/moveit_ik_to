#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
    目标检测的结果
    当作pose
    直接pose调取规划
"""
import sys
import rospy
import moveit_commander
import math
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import JointState
from planner import Planner
from logger import Logger
from publisher import Publisher
from executor import Executor
import time
from utils import angle_to_rad

Point_zero = angle_to_rad([0, 0, 0, 0, 0, 0, 0])
Point_1 = angle_to_rad([ 20, 50, 0,   0, 10,   0, 0])
Point_2 = angle_to_rad([ 30, 90, 0, -50, 90, -30, 0])
Point_3 = angle_to_rad([  0, 90, 0, -50, 90, -30, 0])
Point_4 = angle_to_rad([-35, 10, 0, -30,  0, -30, 0])
Point_5 = angle_to_rad([-35,  0, 0, -20,  0, -50, 0])

rospy.init_node("test_node", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

planner = Planner()
logger = Logger()
publisher = Publisher()
executor = Executor()

# 初始化标志变量
IF_NEW_FLAG = True

# 是否为第一次规划
FIRST_TRAJECTORY_FLAG = True

# 初始化计数器
trajectory_counter = 1
# 最大轨迹次数
MAX_TRAJECTORY_COUNT = 4

# 获取目标姿态的四元数
def robot_euler_from_quaternion(orientation_quaternion):
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_quaternion)

    roll_degrees = np.degrees(roll)
    pitch_degrees = np.degrees(pitch)
    yaw_degrees = np.degrees(yaw)

    print("Roll (degrees):", roll_degrees, "Pitch (degrees):", pitch_degrees, "Yaw (degrees):", yaw_degrees)

    return roll_degrees, pitch_degrees, yaw_degrees

# 调用逆解服务
def calculate_inverse_kinematics(target_pose_stamped):
    ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
    ik_request = GetPositionIKRequest()
    ik_request.ik_request.group_name = 'l_arm_group'  # 设置运动规划组名称
    ik_request.ik_request.pose_stamped = target_pose_stamped  # 设置目标末端位姿
    ik_request.ik_request.timeout = rospy.Duration(0.2)  # 设置逆解计算时间限制

    try:
        ik_response = ik_service(ik_request)
        if ik_response.error_code.val == ik_response.error_code.SUCCESS:
            return ik_response.solution.joint_state
        else:
            rospy.logerr("Inverse kinematics failed with error code: %d", ik_response.error_code.val)
            return None
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))
        return None

# 显示末端逆解的结果
def display_inverse_kinematics_result(target_pose_stamped):
    # 逆解出关节角度
    joint_angles = calculate_inverse_kinematics(target_pose_stamped)
    if joint_angles is None:
        rospy.logerr("Failed to calculate inverse kinematics")
        return

    left_arm_angles_rad   = joint_angles.position[2:9]  # 左手的关节角度
    right_arm_angles_rad  = joint_angles.position[19:26]  # 右手的关节角度

    left_arm_angles_deg = [math.degrees(angle) for angle in left_arm_angles_rad]
    right_arm_angles_deg = [math.degrees(angle) for angle in right_arm_angles_rad]

    print("left_arm_angles_deg : ", left_arm_angles_deg)
    print("right_arm_angles_deg : ", right_arm_angles_deg)

def detection_callback(msg):
    global IF_NEW_FLAG, trajectory_counter

    if not msg.detections:
        rospy.logwarn("No detections in message.")
        return
    
    # 提取目标检测信息（假设只处理第一个检测结果）
    detection = msg.detections[0]
    x = detection.results[0].pose.pose.position.x
    y = detection.results[0].pose.pose.position.y
    z = detection.results[0].pose.pose.position.z  

    target_pose_stamped = PoseStamped()
    target_pose_stamped.header.frame_id = "torso"
    target_pose_stamped.pose.position.x = x
    target_pose_stamped.pose.position.y = y
    target_pose_stamped.pose.position.z = z

    # 修改成斜着抓
    target_pose_stamped.pose.orientation.x = -0.0005388071066334781
    target_pose_stamped.pose.orientation.y = -0.7904212674887817
    target_pose_stamped.pose.orientation.z = 0.00032694187655405566
    target_pose_stamped.pose.orientation.w = 0.6125633213777487

    now_joint_state = planner.get_current_joints_values()
    print("=====================================================")
    print(" now_joint_state : ", now_joint_state)
    print("=====================================================")

    # 规划
    planner.set_start_state(now_joint_state)
    traj = planner.plan_to_target_pose(target_pose_stamped)

    # 发布
    if traj:
        if not IF_NEW_FLAG:
            publisher.start_auto_publish()
            IF_NEW_FLAG = True
        print(" object traj success ! --- now is {0} traj ---".format(trajectory_counter))
        logger.dump_traj(traj, file_name="test1_moveit_point")
        trajectory_counter += 1  # 增加计数器
    else:
        rospy.logerr("Failed to plan trajectory")
        publisher.stop_auto_publish()
        IF_NEW_FLAG = False

    # 计数器
    if trajectory_counter >= MAX_TRAJECTORY_COUNT:
        time.sleep(5)
        rospy.loginfo("Planned 3 successful trajectories, shutting down...")
        rospy.signal_shutdown("Trajectory count limit reached")

    # 执行 等待rviz执行结果
    executor.execute_traj(traj, wait=True)

if __name__ == "__main__":

    logger.make_traj_dir()
    publisher.start_auto_publish()

    print("================= 固定点 轨迹规划 =====================")
    print("=====================================================")
    planner.set_start_state(Point_zero)
    traj = planner.plan_to_target_joints(Point_1)
    logger.dump_traj(traj, file_name="test1")
    executor.execute_traj(traj, wait=True)

    print("=====================================================")
    planner.set_start_state(Point_1)
    traj = planner.plan_to_target_joints(Point_2)
    logger.dump_traj(traj, file_name="test2")
    executor.execute_traj(traj, wait=True)

    print("=====================================================")
    planner.set_start_state(Point_2)
    traj = planner.plan_to_target_joints(Point_3)
    logger.dump_traj(traj, file_name="test3")
    executor.execute_traj(traj, wait=True)

    print("=====================================================")
    planner.set_start_state(Point_3)
    traj = planner.plan_to_target_joints(Point_4)
    logger.dump_traj(traj, file_name="test4")
    executor.execute_traj(traj, wait=True)
    
    print("=====================================================")
    planner.set_start_state(Point_4)
    traj = planner.plan_to_target_joints(Point_5)
    logger.dump_traj(traj, file_name="test5")
    executor.execute_traj(traj, wait=True)

    print("=====================================================")

    # 等待固定轨迹发布完毕
    time.sleep(5)

    # 订阅 /object_yolo_tf2_torso_result 话题
    rospy.Subscriber("/object_yolo_tf2_torso_result", Detection2DArray, detection_callback)

    # 设置频率为 10 Hz
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # 在循环中处理回调
        rate.sleep()

    # 结束自动发布
    publisher.stop_auto_publish()


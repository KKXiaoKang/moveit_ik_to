#!/usr/bin/env python

import rospy
import math
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState
from dynamic_biped.msg import robotArmPose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('end_effector_control')
    rospy.Subscriber("/robot_arm_pose", robotArmPose, pose_callback)
    rospy.spin()

def pose_callback(msg):
    target_pose_stamped = robot_arm_pose_to_pose_stamped(msg)
    joint_angles = calculate_inverse_kinematics(target_pose_stamped)

    if joint_angles is not None:
        # 提取逆解后的结果中左右手的关节角度
        left_arm_angles = joint_angles.position[2:9]  # 左手的关节角度
        right_arm_angles = joint_angles.position[19:26]  # 右手的关节角度

        # 将关节角度转换为弧度
        left_arm_angles_rad = [angle for angle in left_arm_angles]
        right_arm_angles_rad = [angle for angle in right_arm_angles]

        # 创建 JointState 消息
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = [
            'l_arm_pitch', 'l_arm_roll', 'l_arm_yaw', 'l_forearm_pitch', 'l_hand_yaw',
            'l_hand_pitch', 'l_hand_roll', 'r_arm_pitch', 'r_arm_roll', 'r_arm_yaw',
            'r_forearm_pitch', 'r_hand_yaw', 'r_hand_pitch', 'r_hand_roll'
        ]
        joint_state_msg.position = left_arm_angles_rad + right_arm_angles_rad

        # 发布到 /kuavo_arm_traj 话题
        traj_publisher.publish(joint_state_msg)

def robot_arm_pose_to_pose_stamped(msg):
    # 将左手末端位姿数据转换为 PoseStamped 类型
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = "base_link"  # 假设机器人基座坐标系为base_link

    # pose_stamped.pose.position.x = 0.2 
    # pose_stamped.pose.position.y = 0.3 
    # pose_stamped.pose.position.z = 0.1 

    pose_stamped.pose.position.x = msg.left_arm_pose[0]
    pose_stamped.pose.position.y = msg.left_arm_pose[1]
    pose_stamped.pose.position.z = msg.left_arm_pose[2]

    # 将欧拉角转换为四元数
    roll = msg.left_arm_pose[3]
    pitch = msg.left_arm_pose[4]
    yaw = msg.left_arm_pose[5]
    quaternion = quaternion_from_euler(roll, pitch, yaw)

    pose_stamped.pose.orientation.x = quaternion[0]
    pose_stamped.pose.orientation.y = quaternion[1]
    pose_stamped.pose.orientation.z = quaternion[2]
    pose_stamped.pose.orientation.w = quaternion[3]

    return pose_stamped

def calculate_inverse_kinematics(target_pose_stamped):
    # 调用逆解服务
    ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
    ik_request = GetPositionIKRequest()
    ik_request.ik_request.group_name = 'l_arm_group'  # 设置运动规划组名称
    ik_request.ik_request.pose_stamped = target_pose_stamped  # 设置目标末端位姿
    ik_request.ik_request.timeout = rospy.Duration(0.2)  # 设置逆解计算时间限制

    try:
        # 发送逆解请求并等待响应
        ik_response = ik_service(ik_request)
        if ik_response.error_code.val == ik_response.error_code.SUCCESS:
            # 获取逆解结果
            return ik_response.solution.joint_state
        else:
            rospy.logerr("Inverse kinematics failed with error code: %d", ik_response.error_code.val)
            return None
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))
        return None

if __name__ == '__main__':
    try:
        traj_publisher = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
        main()
    except rospy.ROSInterruptException:
        pass

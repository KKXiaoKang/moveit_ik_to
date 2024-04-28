#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState

def main():
    rospy.init_node('end_effector_control')
    rospy.Subscriber("/l_hand_position_end", TransformStamped, pose_callback)
    rospy.spin()

def pose_callback(transform_msg):
    target_pose_stamped = TransformStamped_to_PoseStamped(transform_msg)
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

def TransformStamped_to_PoseStamped(transform_msg):
    pose_stamped = PoseStamped()
    pose_stamped.header = transform_msg.header
    pose_stamped.pose.position.x = transform_msg.transform.translation.x
    pose_stamped.pose.position.y = transform_msg.transform.translation.y
    pose_stamped.pose.position.z = transform_msg.transform.translation.z
    pose_stamped.pose.orientation = transform_msg.transform.rotation
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

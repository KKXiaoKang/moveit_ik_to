#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState

def main():
    rospy.init_node('end_effector_control')

    # 定义目标位姿

    # 做动作/正解末端的位姿 随手截取
    target_pose_stamped = PoseStamped()
    target_pose_stamped.header.frame_id = "torso"
    target_pose_stamped.pose.position.x = 0.21559819053746182
    target_pose_stamped.pose.position.y = 0.26614909246050955
    target_pose_stamped.pose.position.z = 0.11253928177068012
    target_pose_stamped.pose.orientation.x = 0.0
    target_pose_stamped.pose.orientation.y = 0.0
    target_pose_stamped.pose.orientation.z = 0.0
    target_pose_stamped.pose.orientation.w = 1.0
    print(" target_pose_stamped : ", target_pose_stamped)

    # # 目标检测的结果直接拿来用
    # target_pose_stamped = PoseStamped()
    # target_pose_stamped.header.frame_id = "torso"
    # target_pose_stamped.pose.position.x = 0.3263249933369295
    # target_pose_stamped.pose.position.y = 0.04556831815429335
    # target_pose_stamped.pose.position.z = 0.07928416830139445
    # target_pose_stamped.pose.orientation.x = 0.0
    # target_pose_stamped.pose.orientation.y = 0.0
    # target_pose_stamped.pose.orientation.z = 0.0
    # target_pose_stamped.pose.orientation.w = 1.0
    # print(" target_pose_stamped : ", target_pose_stamped)
    
    joint_angles = calculate_inverse_kinematics(target_pose_stamped)

    # 
    left_arm_angles = joint_angles.position[2:9]  # 左手的关节角度
    right_arm_angles = joint_angles.position[19:26]  # 右手的关节角度

    print("left_arm_angles : ", left_arm_angles)
    print("left_arm_angles : ", right_arm_angles)

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

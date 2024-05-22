import rospy
from geometry_msgs.msg import Pose, PoseStamped  # 导入 PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

def main():
    # 初始化 ROS 节点
    rospy.init_node('end_effector_control')

    # 设置目标末端位姿
    target_pose_stamped = PoseStamped()
    target_pose_stamped.header.frame_id = "base_link"  # 设置参考坐标系
    target_pose_stamped.pose.position.x = 0.2  # 设置目标位姿的 x 坐标
    target_pose_stamped.pose.position.y = 0.3  # 设置目标位姿的 y 坐标
    target_pose_stamped.pose.position.z = 0.1  # 设置目标位姿的 z 坐标
    # 设置目标位姿的姿态，例如欧拉角
    target_pose_stamped.pose.orientation.x = 0
    target_pose_stamped.pose.orientation.y = 0
    target_pose_stamped.pose.orientation.z = 0
    target_pose_stamped.pose.orientation.w = 0

    # 调用逆解服务计算关节角度
    joint_angles = calculate_inverse_kinematics(target_pose_stamped)

    # 打印
    print(joint_angles)

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
    main()

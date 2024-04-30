#!/usr/bin/env python
"""
结合MoveIt实现机器人末端位姿控制的方案可以如下：

1. **设置目标位姿**：首先，你需要确定机器人需要移动到的目标位姿，可以通过订阅 `/l_hand_position_end` 和 `/r_hand_position_end` 话题获取当前末端位姿信息，并设置新的目标位姿。

2. **使用MoveIt逆运动学求解器**：MoveIt已经集成了逆运动学求解器，可以通过给定末端位姿来计算机器人的关节角度。你可以使用MoveIt提供的逆运动学接口来实现这一步骤。

3. **生成轨迹**：一旦得到目标关节角度，你可以使用MoveIt提供的运动规划功能来生成一条从当前关节角度到目标关节角度的平滑轨迹。MoveIt使用了规划器来生成路径，确保机器人移动过程中避免碰撞，并且满足运动学和动力学约束。

4. **执行轨迹**：最后，你可以使用MoveIt提供的执行器接口，将生成的轨迹发送给机器人控制器，从而驱动机器人执行移动操作。MoveIt将处理轨迹的执行，监控机器人的状态，并在需要时进行调整，以确保机器人能够准确地跟踪目标位姿。

[ERROR] [1714388142.905979956]: Found empty JointState message
QA : 看起来是无法从逆运动学求解器获取关节角度信息，导致了空的 JointState 消息。这可能是由于逆运动学求解器无法找到有效解决方案，或者请求的姿态无法解析的原因。
* 检查目标位姿是否合理：确保设置的目标位姿在机器人的工作空间内，并且不会导致关节超限或碰撞。
* 检查逆运动学求解器配置：确保已正确配置逆运动学求解器，包括正确指定了机器人的运动学群组和相应的末端执行器链接。
* 检查服务调用：确保服务调用成功，并且返回的响应中包含有效的解决方案。
* 调试消息：可以尝试打印出请求和响应消息，以便进一步调试。
* 可视化工具：使用 RViz 或 MoveIt RViz 插件等可视化工具，查看机器人模型和目标位姿，以便更好地理解问题所在。
"""
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState

def main():
    rospy.init_node('end_effector_control')
    rospy.Subscriber("/l_hand_position_end_fk", PoseStamped, pose_callback)
    rospy.spin()

def pose_callback(pose_stamped_msg):
    joint_angles = calculate_inverse_kinematics(pose_stamped_msg)

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

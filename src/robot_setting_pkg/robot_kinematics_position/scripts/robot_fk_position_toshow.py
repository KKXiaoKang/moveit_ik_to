#!/usr/bin/env python
import rospy
import numpy as np
from dynamic_biped.msg import robotArmInfo
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import Header
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
import matplotlib.pyplot as plt

class WristPositionPublisher:
    def __init__(self):
        rospy.init_node('wrist_position_publisher', anonymous=True)

        # 初始化 tf2 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # 订阅关节状态主题
        self.joint_sub = rospy.Subscriber('/robot_arm_q_v_tau', robotArmInfo, self.joint_callback)

        # 发布末端执行器位姿信息
        self.l_hand_pub = rospy.Publisher('l_hand_position_end_fk', PoseStamped, queue_size=10)
        self.r_hand_pub = rospy.Publisher('r_hand_position_end_fk', PoseStamped, queue_size=10)

        # 初始化运动学正解服务客户端
        self.fk_client = rospy.ServiceProxy('/compute_fk', GetPositionFK)

        # 初始化末端执行器位姿列表
        self.l_hand_pose = []
        self.r_hand_pose = []

    def broadcast_transform(self, pose_stamped, fk_end_id):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = 'torso'
        transform_stamped.child_frame_id = f'fk_end_{fk_end_id}'
        transform_stamped.transform.translation.x = pose_stamped.pose.position.x
        transform_stamped.transform.translation.y = pose_stamped.pose.position.y
        transform_stamped.transform.translation.z = pose_stamped.pose.position.z
        transform_stamped.transform.rotation = pose_stamped.pose.orientation
        
        self.tf_broadcaster.sendTransform(transform_stamped)

    def joint_callback(self, data): 
        # 提取关节角度
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['l_arm_pitch', 'l_arm_roll', 'l_arm_yaw', 'l_forearm_pitch','l_hand_yaw', 'l_hand_pitch', 'l_hand_roll','r_arm_pitch', 'r_arm_roll', 'r_arm_yaw', 'r_forearm_pitch','r_hand_yaw', 'r_hand_pitch', 'r_hand_roll']
        joint_state.position = data.q
    
        # 通过运动学正解服务计算末端执行器位姿
        try:
            fk_request = GetPositionFKRequest()
            fk_request.robot_state.joint_state = joint_state
            fk_request.fk_link_names = ['l_hand_roll', 'r_hand_roll']
            fk_response = self.fk_client(fk_request)
    
            # 发布左手和右手的位姿
            self.l_hand_pub.publish(fk_response.pose_stamped[0])
            self.r_hand_pub.publish(fk_response.pose_stamped[1])

            # 发布位姿广播
            self.broadcast_transform(fk_response.pose_stamped[0], "left")
            self.broadcast_transform(fk_response.pose_stamped[1], "right")

            # 显示末端执行器位姿
            self.l_hand_pose.append(fk_response.pose_stamped[0].pose)
            self.r_hand_pose.append(fk_response.pose_stamped[1].pose)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def visualize_wrist_position(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        l_hand_x = [pose.position.x for pose in self.l_hand_pose]
        l_hand_y = [pose.position.y for pose in self.l_hand_pose]
        l_hand_z = [pose.position.z for pose in self.l_hand_pose]

        r_hand_x = [pose.position.x for pose in self.r_hand_pose]
        r_hand_y = [pose.position.y for pose in self.r_hand_pose]
        r_hand_z = [pose.position.z for pose in self.r_hand_pose]

        ax.plot(l_hand_x, l_hand_y, l_hand_z, label='Left Hand')
        ax.plot(r_hand_x, r_hand_y, r_hand_z, label='Right Hand')

        ax.legend()
        plt.show()

def main():
    wp_publisher = WristPositionPublisher()
    rospy.spin()
    wp_publisher.visualize_wrist_position()

if __name__ == '__main__':
    main()

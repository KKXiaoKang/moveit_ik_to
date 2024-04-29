#!/usr/bin/env python
import rospy
import numpy as np
from dynamic_biped.msg import robotArmQVVD
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

class WristPositionPublisher:
    def __init__(self):
        rospy.init_node('wrist_position_publisher', anonymous=True)

        # 初始化 tf2 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # 订阅关节状态主题
        self.joint_sub = rospy.Subscriber('/robot_arm_q_v_tau', robotArmQVVD, self.joint_callback)

        #
        self.joint_kuavo_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # 发布末端执行器位姿信息
        self.l_hand_pub = rospy.Publisher('l_hand_position_end', geometry_msgs.msg.TransformStamped, queue_size=10)
        self.r_hand_pub = rospy.Publisher('r_hand_position_end', geometry_msgs.msg.TransformStamped, queue_size=10)

    def broadcast_transform(self, pose, tag_id):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'base_link'
        transform_stamped.child_frame_id = f'tag_{tag_id}'
        transform_stamped.transform.translation.x = pose.position.x
        transform_stamped.transform.translation.y = pose.position.y
        transform_stamped.transform.translation.z = pose.position.z
        transform_stamped.transform.rotation = pose.orientation
        
        self.tf_broadcaster.sendTransform(transform_stamped)

    def joint_callback(self, data): 
        #
        print(data)
        
        # 提取关节角度
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['l_arm_pitch', 'l_arm_roll', 'l_arm_yaw', 'l_forearm_pitch','l_hand_yaw', 'l_hand_pitch', 'l_hand_roll','r_arm_pitch', 'r_arm_roll', 'r_arm_yaw', 'r_forearm_pitch','r_hand_yaw', 'r_hand_pitch', 'r_hand_roll']
        #'r_arm_pitch', 'r_arm_roll', 'r_arm_yaw', 'r_forearm_pitch','r_hand_yaw', 'r_hand_pitch', 'r_hand_roll'
        joint_state.position = data.q
        joint_state.velocity = data.v
        joint_state.effort = data.vd

        # 
        self.joint_kuavo_pub.publish(joint_state)
        
        #仅当接收到完整的关节角度数据时处理
        try:
            # 获取左手和右手相对于基座的变换
            transform_l = self.tf_buffer.lookup_transform('base_link', 'l_hand_roll', rospy.Time.now(), rospy.Duration(1.0))
            transform_r = self.tf_buffer.lookup_transform('base_link', 'r_hand_roll', rospy.Time.now(), rospy.Duration(1.0))
            
            # pub 
            self.l_hand_pub.publish(transform_l)
            self.r_hand_pub.publish(transform_r)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF2 error: %s", e)

def main():
    wp_publisher = WristPositionPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()

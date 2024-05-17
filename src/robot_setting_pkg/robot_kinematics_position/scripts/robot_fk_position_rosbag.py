#!/usr/bin/env python
import rospy
import numpy as np
import threading
from dynamic_biped.msg import robotArmInfo
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import Header
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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
        self.time = []
        self.l_hand_x = []
        self.l_hand_y = []
        self.l_hand_z = []

        # 初始化 matplotlib
        self.fig1 = plt.figure()
        self.ax = self.fig1.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])
        self.l_hand_plot, = self.ax.plot([], [], [], label='Left Hand')
        self.r_hand_plot, = self.ax.plot([], [], [], label='Right Hand')
        self.ax.legend()

        # 添加原点标记点
        self.origin_plot, = self.ax.plot([0], [0], [0], 'ro')

        # 创建第二个图形
        self.fig2, (self.ax2, self.ax3, self.ax4) = plt.subplots(3, 1, sharex=True)
        self.ax2.set_ylabel('X')
        self.ax3.set_ylabel('Y')
        self.ax4.set_ylabel('Z')
        self.ax4.set_xlabel('Time')

        # 使用 FuncAnimation 定期更新绘图
        self.ani1 = FuncAnimation(self.fig1, self.update_plot_3d, interval=100)
        self.ani2 = FuncAnimation(self.fig2, self.update_plot_2d, interval=100)

    def update_plot_3d(self, frame):
        if self.l_hand_pose and self.r_hand_pose:
            l_hand_x = [pose.position.x for pose in self.l_hand_pose]
            l_hand_y = [pose.position.y for pose in self.l_hand_pose]
            l_hand_z = [pose.position.z for pose in self.l_hand_pose]

            r_hand_x = [pose.position.x for pose in self.r_hand_pose]
            r_hand_y = [pose.position.y for pose in self.r_hand_pose]
            r_hand_z = [pose.position.z for pose in self.r_hand_pose]

            self.l_hand_plot.set_data(l_hand_x, l_hand_y)
            self.l_hand_plot.set_3d_properties(l_hand_z)

            self.r_hand_plot.set_data(r_hand_x, r_hand_y)
            self.r_hand_plot.set_3d_properties(r_hand_z)

            self.ax.relim()
            self.ax.autoscale_view()

    def update_plot_2d(self, frame):
        if self.time and self.l_hand_x and self.l_hand_y and self.l_hand_z:
            self.ax2.plot(self.time, self.l_hand_x)
            self.ax3.plot(self.time, self.l_hand_y)
            self.ax4.plot(self.time, self.l_hand_z)

            self.ax2.legend()
            self.ax3.legend()
            self.ax4.legend()

    def joint_callback(self, data):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['l_arm_pitch', 'l_arm_roll', 'l_arm_yaw', 'l_forearm_pitch', 'l_hand_yaw', 'l_hand_pitch',
                            'l_hand_roll', 'r_arm_pitch', 'r_arm_roll', 'r_arm_yaw', 'r_forearm_pitch', 'r_hand_yaw',
                            'r_hand_pitch', 'r_hand_roll']
        joint_state.position = data.q

        try:
            fk_request = GetPositionFKRequest()
            fk_request.robot_state.joint_state = joint_state
            fk_request.fk_link_names = ['l_hand_roll', 'r_hand_roll']
            fk_response = self.fk_client(fk_request)

            self.l_hand_pub.publish(fk_response.pose_stamped[0])
            self.r_hand_pub.publish(fk_response.pose_stamped[1])

            self.broadcast_transform(fk_response.pose_stamped[0], "left")
            self.broadcast_transform(fk_response.pose_stamped[1], "right")

            self.l_hand_pose.append(fk_response.pose_stamped[0].pose)
            self.r_hand_pose.append(fk_response.pose_stamped[1].pose)

            # 记录时间和手部位置
            self.time.append(rospy.Time.now().to_sec())
            self.l_hand_x.append(fk_response.pose_stamped[0].pose.position.x)
            self.l_hand_y.append(fk_response.pose_stamped[0].pose.position.y)
            self.l_hand_z.append(fk_response.pose_stamped[0].pose.position.z)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

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

def main():
    wp_publisher = WristPositionPublisher()
    threading.Thread(target=rospy.spin).start()
    plt.show()

if __name__ == '__main__':
    main()
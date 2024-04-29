#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import TransformStamped

def generate_random_pose():
    transform_msg = TransformStamped()
    transform_msg.header.stamp = rospy.Time.now()
    transform_msg.header.frame_id = "base_link"  # 假设机器人基座坐标系为base_link

    # 生成随机位姿
    transform_msg.transform.translation.x = 0.2  # random.uniform(-1.0, 1.0)
    transform_msg.transform.translation.y = 0.3  #random.uniform(-1.0, 1.0)
    transform_msg.transform.translation.z = 0.1  #random.uniform(0.5, 1.5)  # 假设末端位姿在基座上方

    # 设置随机姿态（姿态可以随机生成，或者固定设置）
    transform_msg.transform.rotation.x = 0.0
    transform_msg.transform.rotation.y = 0.0
    transform_msg.transform.rotation.z = 0.0
    transform_msg.transform.rotation.w = 0.0

    return transform_msg

def main():
    rospy.init_node('ik_point_to_test')

    # 创建发布者，发布随机位姿到/l_hand_position_end话题
    pose_publisher = rospy.Publisher('/l_hand_position_end', TransformStamped, queue_size=10)

    rate = rospy.Rate(10)  # 设置发布频率为每秒10次，即每10秒发布一次

    while not rospy.is_shutdown():
        random_pose = generate_random_pose()
        pose_publisher.publish(random_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

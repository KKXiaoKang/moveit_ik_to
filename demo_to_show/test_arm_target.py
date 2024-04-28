#!/usr/bin/env python

import rospy
from dynamic_biped.msg import armTargetPoses  # 替换为你的包名和消息类型

def main():
    # 初始化节点
    rospy.init_node('arm_target_poses_publisher')

    # 创建发布者
    pub = rospy.Publisher('kuavo_arm_target_poses', armTargetPoses, queue_size=10)

    # 定义要发布的数据点
    data_points = [
        (2.0, [20.0, 0, 0, -30, 0, 0, 0, 0, -25, -30, -15, 0, 0, 0]),
        (4.0, [-10.0, 0.0, 10, -40, 0, 0, 0, 0, -75, -80, -15, 0, 0, 0]),
        (6.0, [-25.0, 0.0, 13, -88, 0, 0, 0, 0, -75, -80, -15, 0, 0, 0]),
        (8.0, [-25.0, 0.0, -45, -88, 0, -20, 0, 0, -75, -80, -15, 0, 0, 0]),
        (10.0, [-25.0, 0.0, 13, -88, 0, 0, 0, 0, -75, -80, -15, 0, 0, 0]),
        (12.0, [-25.0, 0.0, -45, -88, 0, -20, 0, 0, -75, -80, -15, 0, 0, 0]),
        (14.0, [-25.0, 0.0, 13, -88, 0, 0, 0, 0, -75, -80, -15, 0, 0, 0]),
        (16.0, [20.0, 0.0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0])
    ]
    
    # 发布数据点
    for time, values in data_points:
        # 等待直到当前时间大于或等于目标时间
        while rospy.get_time() < time:
            rospy.sleep(0.1)  # 以100ms为间隔检查时间

        # 创建 armTargetPoses 消息
        msg = armTargetPoses()
        msg.times = [time]
        msg.values = values

        # 发布消息
        pub.publish(msg)
        rospy.loginfo("Published arm target poses at time %s" % time)

        # 等待一段时间以确保消息被发布
        rospy.sleep(2)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
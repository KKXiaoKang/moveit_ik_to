#!/usr/bin/env python3

import rospy
import sensor_msgs.msg
import moveit_msgs.msg
import geometry_msgs.msg

from base import Base
from utils import rad_to_angle, empty_list

class Publisher(Base):
    """发布轨迹与接收目标位置接口类
    """
    def __init__(self) -> None:
        """发布轨迹接口类初始化
        """
        super(Publisher, self).init_publisher()


    def publish_pose_bias(self, pose_bias: geometry_msgs.msg.PoseStamped) -> None:
        """发布位置偏差

        :param pose_bias: 位置偏移
        """
        self.pose_bias_publisher.publish(pose_bias)


    def publish_l_arm_traj(self, traj: moveit_msgs.msg.RobotTrajectory) -> None:
        """发布左手轨迹

        :param traj: 待发布的轨迹
        """
        joint_state = sensor_msgs.msg.JointState()
        positions  = empty_list(14)
        velocities = empty_list(14)

        rate = rospy.Rate(8)
        for point in traj.joint_trajectory.points:
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = rad_to_angle(point.positions)
            velocities[0:7] = point.velocities
            joint_state.position = positions
            joint_state.velocity = velocities
            
            self.kuavo_arm_traj_publisher.publish(joint_state)
            rate.sleep()


    def publish_r_arm_traj(self, traj: moveit_msgs.msg.RobotTrajectory) -> None:
        """发布右手轨迹

        :param traj: 待发布的轨迹
        """
        joint_state = sensor_msgs.msg.JointState()
        positions  = empty_list(14)
        velocities = empty_list(14)

        rate = rospy.Rate(8)
        for point in traj.joint_trajectory.points:
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[7:14] = rad_to_angle(point.positions)
            velocities[7:14] = point.velocities
            joint_state.position = positions
            joint_state.velocity = velocities
            
            self.kuavo_arm_traj_publisher.publish(joint_state)
            rate.sleep()
    

    def publish_lr_arm_traj(self, l_traj: moveit_msgs.msg.RobotTrajectory, r_traj: moveit_msgs.msg.RobotTrajectory) -> None:
        """发布左右手轨迹

        :param l_traj: 左手轨迹
        :param r_traj: 右手轨迹
        """
        l_traj_points = l_traj.joint_trajectory.points
        r_traj_points = r_traj.joint_trajectory.points
        
        joint_state = sensor_msgs.msg.JointState()
        positions  = empty_list(14)
        velocities = empty_list(14)

        rate = rospy.Rate(8)
        for i in range(min(len(l_traj_points), len(r_traj_points))):
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = rad_to_angle(l_traj_points[i].positions)
            positions[7:14] = rad_to_angle(r_traj_points[i].positions)
            velocities[0:7] = l_traj_points[i].velocities
            velocities[7:14] = r_traj_points[i].velocities
            joint_state.position = positions
            joint_state.velocity = velocities

            self.kuavo_arm_traj_publisher.publish(joint_state)
            rate.sleep()
        
        if len(l_traj_points) > len(r_traj_points):
            for i in range(len(r_traj_points) , len(l_traj_points)):
                if rospy.is_shutdown():
                    rospy.logerr("用户终止程序")
                    exit(0)
                positions[0:7] = rad_to_angle(l_traj_points[i].positions)
                positions[7:14] = rad_to_angle(r_traj_points[-1].positions)
                velocities[0:7] = l_traj_points[i].velocities
                velocities[7:14] = [0 for _ in range(7)]
                joint_state.position = positions
                joint_state.velocity = velocities
                
                self.kuavo_arm_traj_publisher.publish(joint_state)
                rate.sleep()
        else:
            for i in range(len(l_traj_points) , len(r_traj_points)):
                if rospy.is_shutdown():
                    rospy.logerr("用户终止程序")
                    exit(0)
                positions[0:7] = rad_to_angle(l_traj_points[-1].positions)
                positions[7:14] = rad_to_angle(r_traj_points[i].positions)
                velocities[0:7] = [0 for _ in range(7)]
                velocities[7:14] = r_traj_points[i].velocities
                joint_state.position = positions
                joint_state.velocity = velocities
                
                self.kuavo_arm_traj_publisher.publish(joint_state)
                rate.sleep()


    

    
    
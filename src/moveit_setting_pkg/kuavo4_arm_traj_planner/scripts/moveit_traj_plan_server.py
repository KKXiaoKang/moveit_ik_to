#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory
from kuavo4_arm_traj_planner.srv import arm_moveit_joint, arm_moveit_jointRequest, arm_moveit_jointResponse
from kuavo4_arm_traj_planner.srv import arm_moveit_pose, arm_moveit_poseRequest, arm_moveit_poseResponse
from planner import Planner
from utils import angle_to_rad

def handle_pose_request(req, planner):

    response = arm_moveit_poseResponse()
    response.ok_flag = True

    # 创建一个 Pose 对象来存储请求的姿态
    target_pose = Pose()
    target_pose.position.x = req.target_pose.pose.position.x
    target_pose.position.y = req.target_pose.pose.position.y
    target_pose.position.z = req.target_pose.pose.position.z
    target_pose.orientation.x = req.target_pose.pose.orientation.x
    target_pose.orientation.y = req.target_pose.pose.orientation.y
    target_pose.orientation.z = req.target_pose.pose.orientation.z
    target_pose.orientation.w = req.target_pose.pose.orientation.w
    
    # 根据请求的姿态规划轨迹
    if req.arm == 'l':
        robot_traj = planner.goto_l_pose_config(target_pose)
    elif req.arm == 'r':
        robot_traj = planner.goto_r_pose_config(target_pose)
    else:
        rospy.logerr("Invalid arm specified.")
        return response
    
    # 将 RobotTrajectory 转换成 JointTrajectory 消息
    joint_traj = JointTrajectory()
    joint_traj.header.stamp = rospy.Time.now()
    joint_traj.joint_names = robot_traj.joint_trajectory.joint_names
    joint_traj.points = robot_traj.joint_trajectory.points
    
    response.traj = joint_traj
    
    return response

def handle_joint_request(req, planner):
    
    response = arm_moveit_jointResponse()
    response.ok_flag = True

    # 将角度转换为弧度
    joints = angle_to_rad(req.target_joint.position)
    
    # 根据关节位置规划轨迹
    if req.arm == 'l':
        robot_traj = planner.goto_l_joint_config(joints)
    elif req.arm == 'r':
        robot_traj = planner.goto_r_joint_config(joints)
    else:
        rospy.logerr("Invalid arm specified.")
        return response
    
    # 将 RobotTrajectory 转换成 JointTrajectory 消息
    joint_traj = JointTrajectory()
    joint_traj.header.stamp = rospy.Time.now()
    joint_traj.joint_names = robot_traj.joint_trajectory.joint_names
    joint_traj.points = robot_traj.joint_trajectory.points
    
    response.traj = joint_traj
    
    return response


def main():
    rospy.init_node('moveit_traj_plan_server')
    
    # 创建 Planner 实例
    planner = Planner()
    planner.init_arm()
    
    # 创建服务，处理末端姿态请求
    rospy.Service('/kuavo_arm_l_moveit_pose', arm_moveit_pose, lambda req: handle_pose_request(req, planner))
    rospy.Service('/kuavo_arm_r_moveit_pose', arm_moveit_pose, lambda req: handle_pose_request(req, planner))
    
    # 创建服务，处理关节位置请求
    rospy.Service('/kuavo_arm_l_moveit_joint', arm_moveit_joint, lambda req: handle_joint_request(req, planner))
    rospy.Service('/kuavo_arm_r_moveit_joint', arm_moveit_joint, lambda req: handle_joint_request(req, planner))
    
    rospy.spin()

if __name__ == "__main__":
    main()

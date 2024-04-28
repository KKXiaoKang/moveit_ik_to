#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg

from utils import load_config


class Base(object):
    """moveit控制基类
    """
    
    def __init__(self) -> None:
        """moveit控制类初始化
        
        初始化所有控制器、执行器参数和发布结点
        
        config                          : 配置文件
        robot                           : 机器人对象
        scene                           : 场景
        l_move_group                    : 左手规划器
        r_move_group                    : 右手规划器

        display_trajectory_publisher    : 轨迹可视化
        kuavo_arm_traj_publisher        : 轨迹发布

        l_group_name                    : 左手规划组
        r_group_name                    : 右手规划组
        base_frame                      : 基坐标系
        l_planning_frame                : 左手规划坐标系
        r_planning_frame                : 右手规划坐标系
        l_eef_link                      : 左手末端执行器
        r_eef_link                      : 右手末端执行器
        """
        self.config = load_config("./config/kuavo.json")
        if not rospy.core.is_initialized():
            rospy.init_node(self.config["node_name"], anonymous=True)
            moveit_commander.roscpp_initialize(sys.argv)


    def init_move_group(self):
        """初始化规划组
        """
        self.l_group_name = self.config["l_group_name"]
        self.r_group_name = self.config["r_group_name"]

        self.robot = moveit_commander.RobotCommander()
        self.l_move_group = moveit_commander.MoveGroupCommander(self.l_group_name)
        self.r_move_group = moveit_commander.MoveGroupCommander(self.r_group_name)

        self.l_move_group.set_planner_id(self.config["planner_id"])
        self.l_move_group.set_pose_reference_frame(self.config["planning_frame"])
        self.l_move_group.set_num_planning_attempts(self.config["num_planning"])
        self.l_move_group.set_max_acceleration_scaling_factor(self.config["acceleration_scaling_factor"])
        self.l_move_group.set_max_velocity_scaling_factor(self.config["velocity_scaling_factor"])
        self.l_move_group.set_planning_time(self.config["planning_time"])
        self.l_move_group.set_support_surface_name(self.config["support"])
        
        self.r_move_group.set_planner_id(self.config["planner_id"])
        self.r_move_group.set_pose_reference_frame(self.config["planning_frame"])
        self.r_move_group.set_num_planning_attempts(self.config["num_planning"])
        self.r_move_group.set_max_acceleration_scaling_factor(self.config["acceleration_scaling_factor"])
        self.r_move_group.set_max_velocity_scaling_factor(self.config["velocity_scaling_factor"])
        self.r_move_group.set_planning_time(self.config["planning_time"])
        self.r_move_group.set_support_surface_name(self.config["support"])

        self.base_frame = self.robot.get_planning_frame()
        self.l_planning_frame = self.l_move_group.get_planning_frame()
        self.r_planning_frame = self.r_move_group.get_planning_frame()

        self.l_eef_link = self.l_move_group.get_end_effector_link()
        self.r_eef_link = self.r_move_group.get_end_effector_link()


    def init_scene(self):
        """初始化场景
        """
        self.l_group_name = self.config["l_group_name"]
        self.r_group_name = self.config["r_group_name"]

        self.scene = moveit_commander.PlanningSceneInterface()
        self.l_move_group = moveit_commander.MoveGroupCommander(self.l_group_name)
        self.r_move_group = moveit_commander.MoveGroupCommander(self.r_group_name)
        
        self.l_planning_frame = self.l_move_group.get_planning_frame()
        self.r_planning_frame = self.r_move_group.get_planning_frame()

        self.l_eef_link = self.l_move_group.get_end_effector_link()
        self.r_eef_link = self.r_move_group.get_end_effector_link()


    def init_publisher(self):
        """初始化发布器
        """
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        self.kuavo_arm_traj_publisher = rospy.Publisher(
            "/kuavo_arm_traj",
            sensor_msgs.msg.JointState,
            queue_size=20
        )
        self.pose_bias_publisher = rospy.Publisher(
            "/pose_bias",
            geometry_msgs.msg.PoseStamped,
            queue_size=1
        )
    
    

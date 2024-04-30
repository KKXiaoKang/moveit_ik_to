#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from planner import Planner
from utils import dump_traj, angle_to_rad


if __name__ == "__main__":
    rospy.init_node('trajectory_planning_example')

    planner = Planner()
    planner.init_arm()
    

    """动作设计示例 —— 挥手
    
    joints = angle_to_rad([-55, 0, 0, -90, 90, -8, 0])
    traj = planner.goto_r_joint_config(joints)
    dump_traj("traj/hello/traj_r_hello1.json", traj)

    joints = angle_to_rad([-55, -20, 40, -90, 90, -8, 0])
    traj = planner.goto_r_joint_config(joints)
    dump_traj("traj/hello/traj_r_hello2.json", traj)

    joints = angle_to_rad([-55, 10, -10, -90, 90, -8, 0])
    traj = planner.goto_r_joint_config(joints)
    dump_traj("traj/hello/traj_r_hello3.json", traj)

    joints = angle_to_rad([-55, -20, 40, -90, 90, -8, 0])
    traj = planner.goto_r_joint_config(joints)
    dump_traj("traj/hello/traj_r_hello4.json", traj)

    joints = angle_to_rad([-55, 10, -10, -90, 90, -8, 0])
    traj = planner.goto_r_joint_config(joints)
    dump_traj("traj/hello/traj_r_hello5.json", traj)

    joints = angle_to_rad([0, 0, 0, 0, 0, 0, 0])
    traj = planner.goto_r_joint_config(joints)
    dump_traj("traj/hello/traj_r_hello6.json", traj)
    
    """

    """动作设计示例 —— 挥手
    """
    
    # joints = angle_to_rad([-80, 0, 0, -20, 90, 0, 0])
    # traj = planner.goto_r_joint_config(joints)
    # dump_traj("traj/home/traj_r_home1.json", traj)

    # joints = angle_to_rad([20, -10, 30, -85, 0, 0, 0])
    # traj = planner.goto_r_joint_config(joints)
    # dump_traj("traj/home/traj_r_home2.json", traj)

    # joints = angle_to_rad([0, 0, 0, 0, 0, 0, 0])
    # traj = planner.goto_r_joint_config(joints)
    # dump_traj("traj/home/traj_r_home3.json", traj)
    
    # 创建左手末端的轨迹点
    # 0.1 | 0.2 | 0.1
    # 0.2 | 0.3 | 0.1
    # 0.2 | 0.2 | 0.1
    # 0.2 | 0.2 | 0.3
    point = Pose()
    point.position.x = 0.2  # x坐标
    point.position.y = 0.3  # y坐标
    point.position.z = 0.1  # z坐标
    point.orientation.x = 0.0  # 四元数的x分量
    point.orientation.y = 0.0  # 四元数的y分量
    point.orientation.z = 0.0  # 四元数的z分量
    point.orientation.w = 1.0  # 四元数的w分量

    traj = planner.goto_l_pose_config(point)
    dump_traj("traj/point_to_show/traj_point_to_show2.json", traj)

    """查看完整设计轨迹示例 —— 挥手

    traj = planner.goto_r_file_traj_config("traj/hello/traj_r_hello1.json")
    traj = planner.goto_r_file_traj_config("traj/hello/traj_r_hello2.json")
    traj = planner.goto_r_file_traj_config("traj/hello/traj_r_hello3.json")
    traj = planner.goto_r_file_traj_config("traj/hello/traj_r_hello4.json")
    traj = planner.goto_r_file_traj_config("traj/hello/traj_r_hello5.json")
    traj = planner.goto_r_file_traj_config("traj/hello/traj_r_hello6.json")
    
    """

    """
    
    traj = planner.goto_l_file_traj_config("traj/bowing/traj_l_bowing1.json")
    traj = planner.goto_r_file_traj_config("traj/bowing/traj_r_bowing1.json")
    traj = planner.goto_l_file_traj_config("traj/bowing/traj_l_bowing2.json")
    traj = planner.goto_r_file_traj_config("traj/bowing/traj_r_bowing2.json")
    traj = planner.goto_l_file_traj_config("traj/bowing/traj_l_bowing3.json")
    traj = planner.goto_r_file_traj_config("traj/bowing/traj_r_bowing3.json")
    traj = planner.goto_l_file_traj_config("traj/bowing/traj_l_bowing4.json")
    traj = planner.goto_r_file_traj_config("traj/bowing/traj_r_bowing4.json")
    traj = planner.goto_l_file_traj_config("traj/bowing/traj_l_bowing5.json")
    traj = planner.goto_r_file_traj_config("traj/bowing/traj_r_bowing5.json")

    """

    # traj = planner.goto_l_file_traj_config("traj/bowing/traj_l_bowing1.json")
    # traj = planner.goto_r_file_traj_config("traj/bowing/traj_r_bowing1.json")
    # traj = planner.goto_l_file_traj_config("traj/bowing/traj_l_bowing2.json")
    # traj = planner.goto_r_file_traj_config("traj/bowing/traj_r_bowing2.json")
    # traj = planner.goto_l_file_traj_config("traj/bowing/traj_l_bowing3.json")
    # traj = planner.goto_r_file_traj_config("traj/bowing/traj_r_bowing3.json")
    # traj = planner.goto_l_file_traj_config("traj/bowing/traj_l_bowing4.json")
    # traj = planner.goto_r_file_traj_config("traj/bowing/traj_r_bowing4.json")
    # traj = planner.goto_l_file_traj_config("traj/bowing/traj_l_bowing5.json")
    # traj = planner.goto_r_file_traj_config("traj/bowing/traj_r_bowing5.json")
    



















































    """
    =========================================================================================================================

                                                下面的代码是旧版的设计，请勿使用！！！

    =========================================================================================================================
    """





















    """拜年
    
    joints = [-60/180*math.pi, 0/180*math.pi, -35/180*math.pi, -60/180*math.pi, 0/180*math.pi, 0/180*math.pi, -10/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    joints = [-20/180*math.pi, 0/180*math.pi, -35/180*math.pi, -60/180*math.pi, 0/180*math.pi, 0/180*math.pi, -10/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    joints = [-60/180*math.pi, 0/180*math.pi, -35/180*math.pi, -60/180*math.pi, 0/180*math.pi, 0/180*math.pi, -10/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    joints = [-20/180*math.pi, 0/180*math.pi, -35/180*math.pi, -60/180*math.pi, 0/180*math.pi, 0/180*math.pi, -10/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    
    """
    
    
    
    """拜年
    
    traj = planner.goto_l_file_traj_config("traj/bainian/traj_l_bainian1.json")
    traj = l_to_r(traj)
    planner.r_move_group.execute(traj)
    traj = planner.goto_l_file_traj_config("traj/bainian/traj_l_bainian2.json")
    traj = l_to_r(traj)
    planner.r_move_group.execute(traj)
    traj = planner.goto_l_file_traj_config("traj/bainian/traj_l_bainian3.json")
    traj = l_to_r(traj)
    planner.r_move_group.execute(traj)
    traj = planner.goto_l_file_traj_config("traj/bainian/traj_l_bainian4.json")
    traj = l_to_r(traj)
    planner.r_move_group.execute(traj)
    
    """
    
    
    """横幅
    
    joints = [-60/180*math.pi, -10/180*math.pi, -15/180*math.pi, -30/180*math.pi, 0/180*math.pi, -15/180*math.pi, 0/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    joints = [-60/180*math.pi, 10/180*math.pi, 15/180*math.pi, -30/180*math.pi, 0/180*math.pi, 15/180*math.pi, 0/180*math.pi]
    traj = planner.goto_r_joint_config(joints)
    joints = [-30/180*math.pi, 20/180*math.pi, -5/180*math.pi, -30/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    joints = [-100/180*math.pi, -15/180*math.pi, 15/180*math.pi, -30/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi]
    traj = planner.goto_r_joint_config(joints)
    
    """
    
    
    
    
    """横幅
    
    traj = planner.goto_l_file_traj_config("traj/hengfu/traj_l_hengfu1.json")
    traj = planner.goto_r_file_traj_config("traj/hengfu/traj_r_hengfu1.json")
    traj = planner.goto_l_file_traj_config("traj/hengfu/traj_l_hengfu2.json")
    traj = planner.goto_r_file_traj_config("traj/hengfu/traj_r_hengfu2.json")
    
    """
    
    

    
    """取灯笼
    
    joints = [-55/180*math.pi, 0/180*math.pi, 0/180*math.pi, -45/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    joints = [-70/180*math.pi, 0/180*math.pi, 0/180*math.pi, -30/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    joints = [-30/180*math.pi, 0/180*math.pi, 0/180*math.pi, -70/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    joints = [0/180*math.pi, 0/180*math.pi, 0/180*math.pi, -85/180*math.pi, 0/180*math.pi, 0/180*math.pi, -10/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    joints = [0/180*math.pi, 0/180*math.pi, 0/180*math.pi, -50/180*math.pi, 0/180*math.pi, 0/180*math.pi, -10/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    
    """
    
    
    """取灯笼
    
    traj = planner.goto_l_file_traj_config("traj/qudenglong/traj_l_qu1.json")
    traj = planner.goto_l_file_traj_config("traj/qudenglong/traj_l_qu2.json")
    traj = planner.goto_l_file_traj_config("traj/qudenglong/traj_l_qu3.json")
    traj = planner.goto_l_file_traj_config("traj/qudenglong/traj_l_qu4.json")
    traj = planner.goto_l_file_traj_config("traj/qudenglong/traj_l_qu5.json")
    
    """
    
    
    
    """挂灯笼
    
    joints = [0/180*math.pi, 0/180*math.pi, 0/180*math.pi, -50/180*math.pi, 0/180*math.pi, 0/180*math.pi, -10/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    joints = [-70/180*math.pi, 70/180*math.pi, -100/180*math.pi, -60/180*math.pi, 115/180*math.pi, -20/180*math.pi, 30/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    joints = [-145/180*math.pi, 50/180*math.pi, -100/180*math.pi, -60/180*math.pi, 115/180*math.pi, -20/180*math.pi, 30/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    joints = [-140/180*math.pi, 40/180*math.pi, -100/180*math.pi, -60/180*math.pi, 115/180*math.pi, -20/180*math.pi, 30/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    joints = [-140/180*math.pi, 70/180*math.pi, -100/180*math.pi, -60/180*math.pi, 115/180*math.pi, -20/180*math.pi, 30/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    joints = [-140/180*math.pi, 95/180*math.pi, -100/180*math.pi, -60/180*math.pi, 115/180*math.pi, -20/180*math.pi, 30/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    joints = [-70/180*math.pi, 95/180*math.pi, -100/180*math.pi, -60/180*math.pi, 115/180*math.pi, -20/180*math.pi, 30/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    joints = [0/180*math.pi, 0/180*math.pi, 0/180*math.pi, -50/180*math.pi, 0/180*math.pi, 0/180*math.pi, -10/180*math.pi]
    traj = planner.goto_l_joint_config(joints)
    
    """


    """挂灯笼

    traj = planner.goto_l_file_traj_config("traj/guadenglong/traj_l_gua1.json")
    traj = planner.goto_l_file_traj_config("traj/guadenglong/traj_l_gua2.json")
    traj = planner.goto_l_file_traj_config("traj/guadenglong/traj_l_gua3.json")
    traj = planner.goto_l_file_traj_config("traj/guadenglong/traj_l_gua4.json")
    traj = planner.goto_l_file_traj_config("traj/guadenglong/traj_l_gua5.json")
    traj = planner.goto_l_file_traj_config("traj/guadenglong/traj_l_gua6.json")
    traj = planner.goto_l_file_traj_config("traj/guadenglong/traj_l_gua7.json")
    traj = planner.goto_l_file_traj_config("traj/guadenglong/traj_l_gua8.json")
    
    """
    


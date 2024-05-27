#!/usr/bin/env python3
import rospy
import time  # 导入time模块
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from kuavo4_arm_traj_planner.srv import arm_moveit_joint, arm_moveit_pose

ROS_PUBLISH_RATE = 25  # 设置ROS发布频率为25Hz

DEMO_FLAG = "JOINT"  # 设置DEMO模式 : POSE | JOINT

def call_moveit_pose_service(arm, target_pose):
    rospy.wait_for_service('/kuavo_arm_' + arm + '_moveit_pose')
    try:
        moveit_pose_service = rospy.ServiceProxy('/kuavo_arm_' + arm + '_moveit_pose', arm_moveit_pose)
        
        # 记录调用服务前的时间
        start_time = time.time()
        
        response = moveit_pose_service(arm, target_pose)
        
        # 记录调用服务后的时间
        end_time = time.time()
        
        # 计算服务调用时间
        service_time = end_time - start_time
        
        return response.ok_flag, response.traj, service_time
    except rospy.ServiceException as e:
        print("Service call failed:", e)
        return False, [], 0.0

def call_moveit_joint_service(arm, target_joint):
    rospy.wait_for_service('/kuavo_arm_' + arm + '_moveit_joint')
    try:
        moveit_joint_service = rospy.ServiceProxy('/kuavo_arm_' + arm + '_moveit_joint', arm_moveit_joint)
        
        # 记录调用服务前的时间
        start_time = time.time()
        
        response = moveit_joint_service(arm, target_joint)
        
        # 记录调用服务后的时间
        end_time = time.time()
        
        # 计算服务调用时间
        service_time = end_time - start_time
        
        return response.ok_flag, response.traj, service_time
    except rospy.ServiceException as e:
        print("Service call failed:", e)
        return False, [], 0.0

def main():
    rospy.init_node('moveit_client_demo')
    
    # 设置目标姿态
    target_pose = PoseStamped()
    target_pose.header.frame_id = "torso"
    target_pose.pose.position.x = 0.2
    target_pose.pose.position.y = 0.3
    target_pose.pose.position.z = 0.1
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 1.0
    
    # 设置目标关节位置
    target_joint = JointState()
    target_joint.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
    
    arm = 'l'  # 'l' for left arm, 'r' for right arm
    
    rate = rospy.Rate(ROS_PUBLISH_RATE)  # 设置频率为25Hz

    # 设置不同的关节位置列表
    joint_positions_list = [
        [-55, 0, 0, -90, 90, -8, 0],
        [-55, -20, 40, -90, 90, -8, 0],
        [-55, 10, -10, -90, 90, -8, 0],
        [-55, -20, 40, -90, 90, -8, 0],
        [0, 0, 0, 0, 0, 0, 0]
    ]

    # 持续调用
    while not rospy.is_shutdown():
        for joint_positions in joint_positions_list:
            # 设置目标关节位置
            target_joint.position = joint_positions
            
            # TEST
            if DEMO_FLAG == "POSE":
                # 调用笛卡尔坐标系求轨迹
                ok_flag, traj, service_time = call_moveit_pose_service(arm, target_pose)
                if ok_flag:
                    print("Moveit pose service call successful!")
                    print("Received trajectory:", traj)
                    print("Service call time:", service_time)  # 打印服务调用时间
                else:
                    print("Moveit pose service call failed!")
            elif DEMO_FLAG == "JOINT":
                # 调用关节位置求轨迹
                ok_flag, traj, service_time = call_moveit_joint_service(arm, target_joint)
                if ok_flag:
                    print("Moveit joint service call successful!")
                    print("Received trajectory:", traj)
                    print("Service call time:", service_time)  # 打印服务调用时间
                else:
                    print("Moveit joint service call failed!")

            # WAIT FOR RATE
            rate.sleep()  # 控制循环的频率


if __name__ == "__main__":
    main()

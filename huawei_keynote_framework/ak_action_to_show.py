# #!/usr/bin/env python
# # -*- coding: utf-8 -*-
import argparse
import rospy
from kuavoRobotSDK import kuavo
import sys
import os
import math
import termios
import threading
import time
import numpy as np
import rospy
import rospkg
import yaml
import tty
import select

import tf
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import JointState
from std_msgs.msg  import Header

import moveit_commander
from planner import Planner
from logger import Logger
from publisher import Publisher
from executor import Executor
from utils import angle_to_rad
from kuavoRobotSDK import kuavo

Robot_Flag = 1

from dynamic_biped.msg import robotArmQVVD

rospy.init_node("kuavo_action_control_node", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

Point_zero = angle_to_rad([0, 0, 0, 0, 0, 0, 0])
Point_1 = angle_to_rad([ 20, 50, 0,   0, 10,   0, 0])
Point_2 = angle_to_rad([ 30, 90, 0, -50, 90, -30, 0])
Point_3 = angle_to_rad([-15, 90, 0, -50, 45, -40, 0])
Point_4 = angle_to_rad([-50, 50, 0, -30,  0, -50, 0])
Point_5 = angle_to_rad([-50,  0, 0, -30,  0, -50, 0])
Point_Send_water = angle_to_rad([-70,  0, 0, -45,  0, -10, 0])

planner = Planner()
logger = Logger()
publisher = Publisher()
executor = Executor()

DEBUG = False

# 关节角度
joint_state = JointState()
right_arm_state = JointState()
left_arm_state = JointState()

# 初始化标志变量
IF_NEW_FLAG = True
# 是否为第一次规划
FIRST_TRAJECTORY_FLAG = True
# 轨迹失败标志
Failed_TRAJ_FLAG = False
# 失败计数器
Failed_count = 1          
# 初始化计数器
trajectory_counter = 1
# 最大轨迹次数 2 规划一次 / 3 规划2次 / 4 规划3次
MAX_TRAJECTORY_COUNT = 3
# 初始化机器人
robot_instance = kuavo("4_1_kuavo")
# Y轴偏移量
Y_TO_MOVEIT_OFFSET = 0
# 实际要抓取的位置
target_pose_stamped = PoseStamped()

# 定义灵巧手抓取函数
def end_control_to_chosse(kuavo_robot, chosse_flag):
    zero_pose = [0, 0, 0, 0, 0, 0]

    catch_left_pose = [65, 65, 90, 90, 90, 90]
    catch_right_pose = [65, 65, 90, 90, 90, 90]

    open_left_pose = [100, 0, 0, 0, 0, 0]
    open_right_pose = [100, 0, 0, 0, 0, 0]

    if chosse_flag == 0:
        kuavo_robot.set_end_control(zero_pose, zero_pose)
    elif chosse_flag == 1:
        kuavo_robot.set_end_control(catch_left_pose, catch_right_pose)
    elif chosse_flag == 2:
        kuavo_robot.set_end_control(open_left_pose, open_right_pose)

" ---------------------------------- rostopic 订阅 ------------------------------------ "
def detection_callback(msg):
    """
        刷新yolo目标检测的结果
    """
    global IF_NEW_FLAG, trajectory_counter
    global FIRST_TRAJECTORY_FLAG, Failed_count
    global joint_state
    global robot_instance
    global Y_TO_MOVEIT_OFFSET
    global planner, logger, publisher, executor 
    global target_pose_stamped

    # 处理新检测结果
    if not msg.detections:
        rospy.logwarn("No detections in message.")
        return
    
    # 提取目标检测信息（假设只处理第一个检测结果）
    detection = msg.detections[0]
    x = detection.results[0].pose.pose.position.x
    y = detection.results[0].pose.pose.position.y
    z = detection.results[0].pose.pose.position.z  

    target_pose_stamped = PoseStamped()
    target_pose_stamped.header.frame_id = "torso"
    target_pose_stamped.pose.position.x = x
    target_pose_stamped.pose.position.y = (y + Y_TO_MOVEIT_OFFSET)
    target_pose_stamped.pose.position.z = z

    # [ x: -179.9671049, y: -75.5501686, z: -179.8963931 ]
    target_pose_stamped.pose.orientation.x = -0.0005388071066334781
    target_pose_stamped.pose.orientation.y = -0.7904212674887817
    target_pose_stamped.pose.orientation.z = 0.00032694187655405566
    target_pose_stamped.pose.orientation.w = 0.6125633213777487

def joint_callback(data):
    """
        刷新机器人此时手臂的state左手state
    """
    # 提取左手关节角度
    global joint_state
    global right_arm_state
    global left_arm_state

    joint_state.header = Header()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['l_arm_pitch', 'l_arm_roll', 'l_arm_yaw', 'l_forearm_pitch','l_hand_yaw', 'l_hand_pitch', 'l_hand_roll','r_arm_pitch', 'r_arm_roll', 'r_arm_yaw', 'r_forearm_pitch','r_hand_yaw', 'r_hand_pitch', 'r_hand_roll']
    joint_state.position = data.q

    right_arm_state.header = Header()
    right_arm_state.header.stamp = rospy.Time.now()
    right_arm_state.name = ['l_arm_pitch', 'l_arm_roll', 'l_arm_yaw', 'l_forearm_pitch','l_hand_yaw', 'l_hand_pitch', 'l_hand_roll','r_arm_pitch', 'r_arm_roll', 'r_arm_yaw', 'r_forearm_pitch','r_hand_yaw', 'r_hand_pitch', 'r_hand_roll']
    right_arm_state.position = data.q[7:14]

    left_arm_state.header = Header()
    left_arm_state.header.stamp = rospy.Time.now()
    left_arm_state.name = ['l_arm_pitch', 'l_arm_roll', 'l_arm_yaw', 'l_forearm_pitch','l_hand_yaw', 'l_hand_pitch', 'l_hand_roll','r_arm_pitch', 'r_arm_roll', 'r_arm_yaw', 'r_forearm_pitch','r_hand_yaw', 'r_hand_pitch', 'r_hand_roll']
    left_arm_state.position = data.q[0:7]

"--------------------------- moveit 动作规划 菜单 ----------------------------"
def grab_and_deliver_moveit():
    """
    从固定点抓取纯moveit直到把水抓住 + 递水
    """
    print("Executing moveit-based water grabbing and delivery...")
    # 添加具体的moveit操作代码
    pass

def grab_and_deliver_vision():
    """
    从固定点抓取纯视觉微调直到把水抓住 + 递水
    """
    print("Executing vision-based water grabbing and delivery...")
    # 添加具体的视觉抓取操作代码
    pass

def retreat_to_grab_position_left():
    """
    从当前位置回到待抓取的位置（左手）
    """
    global left_arm_state
    global right_arm_state
    global joint_state

    print("Retreating to grab position with left hand...")
    # 添加具体的回退操作代码
    print("=====================================================")
    now_joint_state = left_arm_state.position
    planner.set_start_state(now_joint_state)
    traj = planner.plan_to_target_joints(Point_5)
    executor.execute_traj(traj, wait=True)
    logger.dump_traj(traj, file_name="retreat_to_grab_position_left")
    
def retreat_to_zero_left():
    """
    从当前位置回到0位Zero（左手）
    """
    print("Retreating to zero position with left hand...")
    # 添加具体的回退到0位操作代码
    pass

def retry_grab_vision_left():
    """
    再抓一次视觉抓取（左手）
    """
    print("Retrying vision-based grab with left hand...")
    # 添加具体的视觉抓取操作代码
    pass

def retry_grab_moveit_left():
    """
    再抓一次moveit固定点（左手）
    """
    print("Retrying moveit-based grab with left hand...")
    # 添加具体的moveit抓取操作代码
    pass

def move_to_deliver_position_left():
    """
    从当前位置去到递水位置（左手）
    """
    global left_arm_state
    global right_arm_state
    global joint_state

    print("Moving to water delivery position with left hand...")
    # 添加具体的移动操作代码
    print("=====================================================")
    now_joint_state = left_arm_state.position
    planner.set_start_state(now_joint_state)
    traj = planner.plan_to_target_joints(Point_Send_water)
    executor.execute_traj(traj, wait=True)
    logger.dump_traj(traj, file_name="move_to_deliver_position_left")

def high_five_right():
    """
    击掌（右手）
    """
    print("High fiving with right hand...")
    # 添加具体的击掌操作代码
    pass

"---------------------------keyboard listener----------------------------"
class keyboardlinstener(object):
    def __init__(self):
        super(keyboardlinstener,self).__init__()
        self.key_val = ""
        self.update = False

    def getKey(self, key_timeout):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ' '
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

"--------------------------- menu list -----------------------------------"
def menu():
    print("选择要机器人执行操作的动作,直接点击对应序号即可,无需按下Enter:")
    print("1. 全程 -- 从固定点抓取纯moveit直到把水抓住 + 递水")
    print("2. 全程 -- 从固定点抓取纯视觉微调直到把水抓住 + 递水")
    print("3. 回退 -- 从当前位置回到待抓取的位置（左手） ")
    print("4. 回退 -- 从当前位置回到0位Zero（左手） ")
    print("5. 再抓一次 -- 再抓一次视觉抓取（左手） ")
    print("6. 再抓一次 -- 再抓一次moveit固定点（左手） ")
    print("7. 纯动作 -- 从当前位置去到递水位置（左手） ")
    print("8. 纯动作 -- 击掌（右手） ")
    print("----------")
    print("9. 退出机器人动作操作")

def main():
    end_control_to_chosse(robot_instance, 0)

    logger.make_traj_dir()
    publisher.start_auto_publish()

    parser = argparse.ArgumentParser(description="Kuavo Robot Control Script")
    args = parser.parse_args()

    robot = kuavo("kuavo_robot")  # 请替换成实际的机器人名字
    key_lisnter = keyboardlinstener()

    # 订阅
    joint_sub = rospy.Subscriber('/robot_arm_q_v_tau', robotArmQVVD, joint_callback)
    
    # 订阅 /object_yolo_tf2_torso_result 话题
    yolov_sub = rospy.Subscriber("/object_yolo_tf2_torso_result", Detection2DArray, detection_callback)
    

    # 等待3s topic 激活
    time.sleep(3)

    # 从当前位置回到待抓取的位置（左手）
    retreat_to_grab_position_left()

    # move_to_deliver_position_left
    move_to_deliver_position_left()

    # while True:
    #     print('\033c')
    #     menu()
        
    #     # choice = input("请输入选择的操作编号 (6 退出): ")
    #     choice = key_lisnter.getKey(0.1)

    #     if choice == 'q':
    #         break

    #     try:
    #         choice = int(choice)
    #     except ValueError:
    #         print("无效的选择，请输入数字。")
    #         continue
    #     if choice == '1':    # 从固定点抓取纯moveit直到把水抓住 + 递水
    #         grab_and_deliver_moveit()
    #     elif choice == '2':  # 从固定点抓取纯视觉微调直到把水抓住 + 递水
    #         grab_and_deliver_vision()
    #     elif choice == '3':  # 从当前位置回到待抓取的位置（左手）
    #         retreat_to_grab_position_left()
    #     elif choice == '4':  # 从当前位置回到0位Zero（左手）
    #         retreat_to_zero_left()
    #     elif choice == '5':  # 再抓一次视觉抓取（左手）
    #         retry_grab_vision_left()
    #     elif choice == '6':  # 再抓一次moveit固定点（左手）
    #         retry_grab_moveit_left()
    #     elif choice == '7':  # 从当前位置去到递水位置（左手）
    #         move_to_deliver_position_left()
    #     elif choice == '8':  # 击掌（右手）
    #         high_five_right()    
    #     elif choice == 9:
    #         # 退出遥控操作
    #         print("正在退出机器人执行操作...请稍后")
    #         break         
    #     else:
    #         print("无效的选择, 请选择1-9之间的数字。")

if __name__ == "__main__":
    main()

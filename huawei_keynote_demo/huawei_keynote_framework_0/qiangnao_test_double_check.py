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
from std_msgs.msg import *
from geometry_msgs.msg import *
import tty
import select
from dynamic_biped.msg import robotHeadMotionData

hand_max_left_pose = [30.0, -10.0]
hand_max_right_pose = [-30.0, -10.0]
hand_zero_pose = [0.0, 0.0]
hand_up_pose = [0.0, -20.0]

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

def call_head_motion_data_pub(pub, head_motion_data):
    msg = robotHeadMotionData()
    msg.joint_data = head_motion_data
    pub.publish(msg)
    time.sleep(2)

# 线性插值
def line_to_end_hand_control(kuavo_robot):

    # 合上
    for i in range(65):
        kuavo_robot.set_end_control([i, i, i, i, i, i], [i, i, i, i, i, i])
        time.sleep(0.2)
        print(" control end Hand close : ", i)

    # 打开
    j = 65
    for i in range(65):
        kuavo_robot.set_end_control([j, j, j, j, j, j], [j, j, j, j, j, j])
        time.sleep(0.2)
        j = j - 1
        print(" control end Hand open : ", j)        
    
# 定义灵巧手抓取函数
def end_control_to_chosse(kuavo_robot, chosse_flag):
    zero_pose = [0, 0, 0, 0, 0, 0]

    catch_left_pose = [65, 65, 90, 90, 90, 90]
    catch_right_pose = [65, 65, 90, 90, 90, 90]

    open_left_pose = [100, 0, 0, 0, 0, 0]
    open_right_pose = [0, 100, 0, 0, 0, 0]

    thumb_up_left_pose =  [0, 0, 80, 80, 80, 80]
    thumb_up_right_pose = [0, 0, 80, 80, 80, 80]
    
    ok_left_pose =  [65, 65, 90, 0, 0, 0]
    ok_right_pose = [65, 65, 90, 0, 0, 0]
    
    if chosse_flag == 0:
        kuavo_robot.set_end_control(zero_pose, zero_pose)
    elif chosse_flag == 1:
        kuavo_robot.set_end_control(catch_left_pose, catch_right_pose)
    elif chosse_flag == 2:
        kuavo_robot.set_end_control(open_left_pose, open_right_pose)
    elif chosse_flag == 3:
        kuavo_robot.set_end_control(thumb_up_left_pose, thumb_up_right_pose)
    elif chosse_flag == 4:
        kuavo_robot.set_end_control(ok_left_pose, ok_right_pose)

def menu():
    print("选择要灵巧手操作或者头部的动作,直接点击对应序号即可,无需按下Enter:")
    print("1. 灵巧手 -- 完全张开")
    print("2. 灵巧手 -- 打开虎口")
    print("3. 灵巧手 -- 完全关闭")
    print("4. 头部 -- 扭头左边")
    print("5. 头部 -- 扭头右边")
    print("6. 头部 -- 微微抬头")
    print("7. 头部 -- 回到0位")
    print("8. 灵巧手 -- 点赞")
    print("0. 灵巧手 -- ok手势")
    print("----------")
    print("9. 退出灵巧手操作或者头部操作")

def main():
    rospy.init_node("test_control_node")
    parser = argparse.ArgumentParser(description="Kuavo Robot Control Script")
    args = parser.parse_args()

    robot = kuavo("kuavo_robot")  # 请替换成实际的机器人名字
    key_lisnter = keyboardlinstener()
    pub = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=10)
    
    while not rospy.is_shutdown():
        # time.sleep(2)
        # end_control_to_chosse(robot, 3)

        # time.sleep(2)
        # end_control_to_chosse(robot, 2)

        # time.sleep(2)
        # end_control_to_chosse(robot, 1)
        line_to_end_hand_control(robot)
        
if __name__ == "__main__":
    main()

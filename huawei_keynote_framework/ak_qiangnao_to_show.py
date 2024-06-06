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

def menu():
    print("选择要灵巧手操作的动作,直接点击对应序号即可,无需按下Enter:")
    print("1. 完全张开")
    print("2. 打开虎口")
    print("3. 完全关闭")
    print("----------")
    print("9. 退出灵巧手操作")

def main():
    rospy.init_node("------欢迎来到灵巧手控制脚本-------------")
    parser = argparse.ArgumentParser(description="Kuavo Robot Control Script")
    args = parser.parse_args()

    robot = kuavo("kuavo_robot")  # 请替换成实际的机器人名字
    key_lisnter = keyboardlinstener()

    while True:
        print('\033c')
        menu()
        # choice = input("请输入选择的操作编号 (6 退出): ")
        choice = key_lisnter.getKey(0.1)

        if choice == 'q':
            break

        try:
            choice = int(choice)
        except ValueError:
            print("无效的选择，请输入数字。")
            continue
        if choice == 1:   # 完全张开
            end_control_to_chosse(robot, 0)
        elif choice == 2: # 打开虎口
            end_control_to_chosse(robot, 2)
        elif choice == 3: # 完全关闭
            end_control_to_chosse(robot, 1)
        elif choice == 9:
            # 退出遥控操作
            print("正在退出灵巧手操作...请稍后")
            break         
        else:
            print("无效的选择, 请选择1-9之间的数字。")

if __name__ == "__main__":
    main()

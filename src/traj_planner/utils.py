#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import json
import rospy
import numpy as np


def load_json(path: str) -> dict:
    """ 加载json文件 """
    with open(path, "r") as f:
        json_data = json.load(f)
    rospy.loginfo("json已从{}中加载".format(path))
    
    return json_data


def rad_to_angle(rad_list: list) -> list:
    """ 弧度转变为角度 """
    return (np.array(rad_list)/np.pi*180).tolist()


def angle_to_rad(angle_list: list) -> list:
    """ 角度转变为弧度 """
    return (np.array(angle_list)/180*np.pi).tolist()


def check_num(num: float) -> bool:
    """ 检查输入数字 """
    if str(num).isdigit() and num > 0:
        return True
    else:
        return False

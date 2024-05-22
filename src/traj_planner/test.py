#!/usr/bin/env python3
# -*- coding: UTF-8 -*-


import sys
import moveit_commander
import rospy

from planner import Planner
from logger import Logger
from publisher import Publisher

from utils import angle_to_rad

Point_zero = angle_to_rad([0, 0, 0, 0, 0, 0, 0])

Point_1 = angle_to_rad([ 40, 40, 0,   0,  0, -60, 0])
Point_2 = angle_to_rad([ 30, 70, 0, -50, 90, -30, 0])
Point_3 = angle_to_rad([-30, 10, 0, -30,  0, -30, 0])
Point_4 = angle_to_rad([  0,  0, 0, -20,  0, -70, 0])

if __name__ == "__main__":
    rospy.init_node("test_node", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    planner = Planner()
    logger = Logger()
    publisher = Publisher()
    
    publisher.start_auto_publish()
    logger.make_traj_dir()

    print("=====================================================")
    planner.set_start_state(Point_zero)
    traj = planner.plan_to_target_joints(Point_1)
    logger.dump_traj(traj, file_name="test1")

    print("=====================================================")
    planner.set_start_state(Point_1)
    traj = planner.plan_to_target_joints(Point_2)
    logger.dump_traj(traj, file_name="test2")


    print("=====================================================")
    planner.set_start_state(Point_2)
    traj = planner.plan_to_target_joints(Point_3)
    logger.dump_traj(traj, file_name="test3")


    print("=====================================================")
    planner.set_start_state(Point_3)
    traj = planner.plan_to_target_joints(Point_4)
    logger.dump_traj(traj, file_name="test4")
    
    publisher.stop_auto_publish()

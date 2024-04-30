#!/usr/bin/env python

import rospy
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
import random

def sample_joint_values(joint_limits):
    """
    Randomly sample joint values within joint limits.
    """
    joint_values = []
    for limit in joint_limits:
        joint_values.append(random.uniform(limit[0], limit[1]))
    return joint_values

def estimate_reachable_workspace(group_name, num_samples=1000):
    """
    Estimate reachable workspace using Monte Carlo method.
    """
    rospy.wait_for_service('/get_planning_scene')
    try:
        get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        req = PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
        res = get_planning_scene(req)
        joint_limits = res.robot_state.joint_state.position
        joint_values = []

        for _ in range(num_samples):
            joint_values.append(sample_joint_values(joint_limits))

        return joint_values

    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == '__main__':
    rospy.init_node('reachable_workspace_estimator')
    
    l_arm_workspace = estimate_reachable_workspace('l_arm_group')
    r_arm_workspace = estimate_reachable_workspace('r_arm_group')

    print("Left arm reachable workspace samples:", l_arm_workspace)
    print("Right arm reachable workspace samples:", r_arm_workspace)

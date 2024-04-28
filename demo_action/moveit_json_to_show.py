"""
    轨迹查看器
"""
import rospy
import sensor_msgs.msg
import time
from utils import rad_to_angle, l_to_r, load_traj
import os
import json
import rospy_message_converter

positions = []
velocities = []
accelerations = []

#moveit_file_str = "./traj/home/traj_r_home3.json"     # acceleration_scaling_factor 1 | velocity_scaling_factor 1      | 26 length
#moveit_file_str = "./traj/home_1_0/traj_r_home3.json" # acceleration_scaling_factor 0.1 | velocity_scaling_factor 0.1  | 26 length
moveit_file_str = "./traj/home_time_5/traj_r_home3.json" 

def load_traj(path: str):
    """
        加载轨迹
    """
    path = os.path.join(os.path.dirname(__file__), path)
    with open(path, "r") as f:
        traj = json.load(f)
    traj = rospy_message_converter.json_message_converter.convert_json_to_ros_message("moveit_msgs/RobotTrajectory", traj)
    rospy.loginfo("轨迹已从{}中加载".format(path))
    return traj

def append_data_to_list(traj):
    """
        将轨迹数据添加到列表中
    """
    global positions, velocities, accelerations

    for point in traj.joint_trajectory.points:
        if rospy.is_shutdown():
            rospy.logerr("用户终止程序")
            exit(0)
        positions.append(rad_to_angle(point.positions))
        velocities.append(point.velocities)
        accelerations.append(point.accelerations)

def plot_to_show():
    import matplotlib.pyplot as plt

    # 解析轨迹数据
    timestamps = [i for i in range(len(positions))]  # 生成时间戳，假设时间戳从0开始递增

    # 绘制关节位置随时间的变化
    plt.figure(figsize=(10, 6))
    for i in range(len(positions[0])):
        plt.plot(timestamps, [pos[i] for pos in positions], label=f'Joint {i+1}')
    plt.xlabel('Time')
    plt.ylabel('Joint Position (degrees)')
    plt.title('Joint Positions vs Time')
    plt.legend()
    plt.grid(True)

    # 绘制关节速度随时间的变化
    plt.figure(figsize=(10, 6))
    for i in range(len(velocities[0])):
        plt.plot(timestamps, [vel[i] for vel in velocities], label=f'Joint {i+1}')
    plt.xlabel('Time')
    plt.ylabel('Joint Velocity (rad/s)')
    plt.title('Joint Velocities vs Time')
    plt.legend()
    plt.grid(True)
    
    # 绘制关节加速度随时间的变化
    plt.figure(figsize=(10, 6))
    for i in range(len(accelerations[0])):
        plt.plot(timestamps, [acc[i] for acc in accelerations], label=f'Joint {i+1}')
    plt.xlabel('Time')
    plt.ylabel('Joint Acceleration (rad/s^2)')
    plt.title('Joint Accelerations vs Time')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":

    rospy.init_node("moveit_json_to_show")

    result = load_traj(moveit_file_str)

    append_data_to_list(result)

    print(" moveit " + moveit_file_str + " loaded length :", len(positions))

    plot_to_show()
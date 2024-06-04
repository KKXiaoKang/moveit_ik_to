import sys
import os
import time
import math
# sys.path.append("/usr/lib/python3/dist-packages")
import numpy as np
import pydrake

from pydrake.all import (
    AddMultibodyPlantSceneGraph, DiagramBuilder, Parser,PiecewisePolynomial
)
from pydrake.all import StartMeshcat, AddMultibodyPlantSceneGraph, MeshcatVisualizer

sys.path.append(os.path.abspath(os.path.dirname(__file__) + r'../../../'))
from scripts.common.IK import *
from scripts.common.utils import *

from scipy.spatial.transform import Rotation as R

import rospy
from vision_msgs.msg import Detection2DArray, Detection2D
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class ArmIk:
    def __init__(self,model_file, end_frames_name):
        builder = DiagramBuilder()
        self.__plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
        parser = Parser(self.__plant)
        robot = parser.AddModelFromFile(model_file)
        self.__plant.Finalize()

        self.__visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
        self.__diagram = builder.Build()
        self.__diagram_context = self.__diagram.CreateDefaultContext()

        self.__plant_context = self.__plant.GetMyContextFromRoot(self.__diagram_context)
        self.__q0 = self.__plant.GetPositions(self.__plant_context)
        self.__v0 = self.__plant.GetVelocities(self.__plant_context)
        self.__r0 = self.__plant.CalcCenterOfMassPositionInWorld(self.__plant_context)

        self.__base_link_name = end_frames_name[0]
        self.__left_eef_name = end_frames_name[1]
        self.__right_eef_name = end_frames_name[2]

        self.__IK = TorsoIK(self.__plant, end_frames_name, 1e-4, 1e-4)
    
    def q0(self):
        return self.__q0
    
    def init_state(self, torso_yaw_deg, torso_height):
        self.__torso_yaw_rad = math.radians(torso_yaw_deg)       
        self.__torso_height = torso_height       
        self.__q0[6] = torso_height

    def computeIK(self, q0, l_hand_pose, r_hand_pose, l_hand_RPY=None, r_hand_RPY=None):
            torsoR = [0.0, self.__torso_yaw_rad, 0.0]
            r = [0.0, 0.0, self.__torso_height]
            
            pose_list = [
                [torsoR, r],
                [l_hand_RPY, l_hand_pose],
                [r_hand_RPY, r_hand_pose],
            ]
            is_success, q = self.__IK.solve(pose_list, q0=q0)
            if not is_success:
                print(f"pose: {pose_list[0][0]}, {pose_list[0][1]}")
                print(f"lhand: {pose_list[1][0]}, {pose_list[1][1]}")
                print(f"rhand: {pose_list[2][0]}, {pose_list[2][1]}")
                # raise RuntimeError("Failed to IK0!")
                return None
            else:
                return q 

    def start_recording(self):
        self.__visualizer.StartRecording()

    def stop_andpublish_recording(self):
        self.__visualizer.StopRecording()
        self.__visualizer.PublishRecording()

    def visualize_animation(self, q_list, start_time=0.0, duration=1.1):
        t_sol = np.arange(start_time, start_time+duration, 1)  
        q_sol = np.array(q_list).T
        # print(f"q_sol: {q_sol.shape}, t_sol: {t_sol.shape}")
        q_pp = PiecewisePolynomial.FirstOrderHold(t_sol, q_sol)
        t0 = t_sol[0]
        tf = t_sol[-1]
        t = t0
        # self.__visualizer.StartRecording()
        while t < tf:
            q = q_pp.value(t)
            self.__plant.SetPositions(self.__plant_context, q)
            self.__diagram_context.SetTime(t)
            self.__diagram.ForcedPublish(self.__diagram_context)
            t += 0.01
        # self.__visualizer.StopRecording()
        # self.__visualizer.PublishRecording()
        # while True:
        time.sleep(0.1)
    
    def left_hand_jacobian(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        J_hand_in_world = self.__plant.CalcJacobianSpatialVelocity(
            self.__plant_context, JacobianWrtVariable.kV,
            self.__plant.GetFrameByName(self.__left_eef_name), [0, 0, 0], self.__plant.world_frame(), self.__plant.world_frame())
        return J_hand_in_world

    def right_hand_jacobian(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        J_hand_in_world = self.__plant.CalcJacobianSpatialVelocity(
            self.__plant_context, JacobianWrtVariable.kV,
            self.__plant.GetFrameByName(self.__right_eef_name), [0, 0, 0], self.__plant.world_frame(), self.__plant.world_frame())
        return J_hand_in_world
    
    def left_hand_pose(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        l_hand_in_base = self.__plant.GetFrameByName(self.__left_eef_name).CalcPose(self.__plant_context, self.__plant.GetFrameByName(self.__base_link_name))
        print("left hand position in base:", l_hand_in_base.translation())
        return l_hand_in_base
    
    def right_hand_pose(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        r_hand_in_base = self.__plant.GetFrameByName(self.__right_eef_name).CalcPose(self.__plant_context, self.__plant.GetFrameByName(self.__base_link_name))
        print("left hand position in base:", r_hand_in_base.translation())
        return r_hand_in_base


i = 0
meshcat = StartMeshcat()
model_file = "robots/biped_s4_ik/urdf/biped_s4_ik_moveit.urdf"
base_link_name = 'torso'
end_frames_name = [base_link_name,'l_hand_end_virtual','r_hand_end_virtual']
arm_ik = ArmIk(model_file, end_frames_name)
torso_yaw_deg = 0.0
torso_height = 0.0
arm_ik.init_state(torso_yaw_deg, torso_height)
q0 = arm_ik.q0()
q_list = [q0]
last_q = q0
arm_ik.start_recording()
t = 0.0
l_pose = arm_ik.left_hand_pose(q0)
print(f"left_hand_pose: {l_pose.translation()}, {l_pose.rotation()}")

def detection_callback(msg):
    global t
    global arm_ik
    global q0
    global last_q
    global q_list

    if not msg.detections:
        rospy.logwarn("No detections in message.")
        return
    
    # 提取目标检测信息（假设只处理第一个检测结果）
    detection = msg.detections[0]
    x = detection.results[0].pose.pose.position.x
    y = detection.results[0].pose.pose.position.y
    z = detection.results[0].pose.pose.position.z
    print(f"detection: {x}, {y}, {z}")
    """
    moveit 冗余度0.15 可以解算出来位置
    
    detection: 0.4839796909273487, 0.15725233281251966, 0.11794171053813857
    x = (x-0.05) 
    """
    # x = (x-0.03) 
    
    l_hand_pose = [x, y, z] # 不计算就给None
    l_hand_RPY = None

    r_hand_RPY = None
    r_hand_pose = None

    time_0 = time.time()
    q = arm_ik.computeIK(q0, l_hand_pose, r_hand_pose, l_hand_RPY, r_hand_RPY)
    time_cost = time.time() - time_0
    print(f"time cost: {1e3*time_cost:.3f} ms")

    # print(f" success drake ik q: {q}")
    if q is None:
        print(f"Failed to IK in step {i}!")
    else:
        # IK 解释
        print(" ------------------ ")
        print(f" success drake ik q: {q}") 

        # 取对应的关节角度
        joint_state_position = q[7:14]   
        print(f" joint_state_position: {joint_state_position}")     
    
    # if q is not None:
    #     q_list.append(q)
    #     q0 = q
    #     # animate trajectory
    #     arm_ik.visualize_animation([last_q, q], t)
    #     last_q = q
    #     t = t + 1.0
    #     print(f" success drake ik q: {q}")
    # else:
    #     print(f"Failed to IK in step {i}!")

if __name__ == "__main__":
    rospy.init_node("drake_ik_node", anonymous=True)

    # （1）数据加载配置
    test = np.load("./rosbag_joint.npy") # drake RPY版本
    # test = np.load("./rosbag_s.npy") # 四元数版本（x,y,z,w）
    np.set_printoptions(linewidth=240)
    np.set_printoptions(threshold=2000)
    np.set_printoptions(precision=4)
    np.set_printoptions(suppress=True)

    # （2） ros话题配置
    yolo_object_result_pub = rospy.Subscriber('/object_yolo_tf2_torso_result', Detection2DArray, detection_callback)

    # （4） 等待循环
    rospy.spin()

    # （5） end 结束
    arm_ik.stop_andpublish_recording()
    print('Program end, Press Ctrl + C to exit.')

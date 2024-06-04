import numpy as np
from functools import partial
from pydrake.all import (InverseKinematics, Solve, RotationMatrix, RollPitchYaw, SnoptSolver,InitializeAutoDiff,JacobianWrtVariable,
                         ExtractGradient, ExtractValue, ComPositionConstraint, CentroidalMomentumConstraint)


from scipy.spatial.transform import Rotation as R

class TorsoIK:
    def __init__(self, plant, frame_name_list, constraint_tol=1.0e-8, solver_tol=1.0e-6):
        self.__plant = plant
        self.__plant_context = self.__plant.CreateDefaultContext()
        self.__constraint_tol = constraint_tol
        self.__solver_tol = solver_tol
        self.__frames = [self.__plant.GetFrameByName(name) for name in frame_name_list]
        '''
        self.__IK = InverseKinematics(plant=self.__plant, with_joint_limits=False)
        transform = [frame.CalcPose(self.__plant_context, self.__frames[0]) for frame in self.__frames]
        self.__R_constraint = []
        self.__p_constraint = []
        for i, frame in enumerate(self.__frames):
            self.__R_constraint.append(self.__IK.AddOrientationConstraint(self.__plant.world_frame(), transform[i].rotation(),
                                                                          frame, RotationMatrix(RollPitchYaw(0, 0, 0)),
                                                                          self.__constraint_tol).evaluator())
            self.__p_constraint.append(self.__IK.AddPositionConstraint(frameB=frame,
                                                                       p_BQ=np.zeros(3),
                                                                       frameA=self.__plant.world_frame(),
                                                                       p_AQ_upper=transform[i].translation()+self.__constraint_tol,
                                                                       p_AQ_lower=transform[i].translation()-self.__constraint_tol).evaluator())
        '''

    def solve(self, pose_list, q0=[]):
        self.__IK = InverseKinematics(plant=self.__plant, with_joint_limits=False)
        for i, frame in enumerate(self.__frames):
            if not pose_list[i][0] is None:
                self.__IK.AddOrientationConstraint(self.__plant.world_frame(), RotationMatrix(RollPitchYaw(pose_list[i][0])),
                                            frame, RotationMatrix(RollPitchYaw(0, 0, 0)),
                                            self.__constraint_tol)
            if not pose_list[i][1] is None:
                self.__IK.AddPositionConstraint(frameB=frame,
                                                p_BQ=np.zeros(3),
                                                frameA=self.__plant.world_frame(),
                                                p_AQ_lower=np.array(pose_list[i][1])-self.__constraint_tol,
                                                p_AQ_upper=np.array(pose_list[i][1])+self.__constraint_tol)
        result = Solve(self.__IK.prog(), q0)
        if result.is_success():
            return [True, result.GetSolution()]
        return [False, []]


class CoMIK:
    def __init__(self, plant, frame_name_list, constraint_tol=1.0e-8, solver_tol=1.0e-6):
        self.__plant = plant
        self.__plant_context = self.__plant.CreateDefaultContext()
        self.__constraint_tol = constraint_tol
        self.__solver_tol = solver_tol
        self.__frames = [self.__plant.GetFrameByName(name) for name in frame_name_list]
        '''
        self.__IK = InverseKinematics(plant=self.__plant, with_joint_limits=False)
        transform = [frame.CalcPose(self.__plant_context, self.__frames[0]) for frame in self.__frames]
        r0 = self.__plant.CalcCenterOfMassPositionInWorld(self.__plant_context)
        self.__R_constraint = []
        self.__p_constraint = []
        for i, frame in enumerate(self.__frames):
            self.__R_constraint.append(self.__IK.AddOrientationConstraint(self.__plant.world_frame(), transform[i].rotation(),
                                                                          frame, RotationMatrix(RollPitchYaw(0, 0, 0)),
                                                                          self.__constraint_tol).evaluator())
            if i == 0:
                self.__p_constraint.append(self.__CoMPositionConstraint(self.__IK, r0).evaluator())
            else:
                self.__p_constraint.append(self.__IK.AddPositionConstraint(frameB=frame,
                                                                           p_BQ=np.zeros(3),
                                                                           frameA=self.__plant.world_frame(),
                                                                           p_AQ_upper=transform[i].translation()+self.__constraint_tol,
                                                                           p_AQ_lower=transform[i].translation()-self.__constraint_tol).evaluator())
        '''

    def __CoMPositionConstraint(self, IK, r_des):
        com_position_constraint = ComPositionConstraint(self.__plant, None, self.__plant.world_frame(), self.__plant_context)
        r = IK.prog().NewContinuousVariables(3)  # r is the variable for CoM position.
        IK.prog().AddConstraint(com_position_constraint, np.concatenate([IK.q(), r]))
        b = IK.prog().AddBoundingBoxConstraint(r_des, r_des, r)
        IK.prog().SetInitialGuess(r, r_des)
        return b

    def solve(self, pose_list, q0=[]):
        # use joint limits, all joints should be given limits
        # self.__IK = InverseKinematics(plant=self.__plant, with_joint_limits=True)

        self.__IK = InverseKinematics(plant=self.__plant, with_joint_limits=False)
        for i, frame in enumerate(self.__frames):
            if not pose_list[i][0] is None:
                self.__IK.AddOrientationConstraint(self.__plant.world_frame(), RotationMatrix(RollPitchYaw(pose_list[i][0])),
                                                   frame, RotationMatrix(RollPitchYaw(0, 0, 0)),
                                                   self.__constraint_tol)
            if not pose_list[i][1] is None:
                if i == 0:
                    self.__CoMPositionConstraint(self.__IK, pose_list[i][1])
                else:
                    self.__IK.AddPositionConstraint(frameB=frame,
                                                    p_BQ=np.zeros(3),
                                                    frameA=self.__plant.world_frame(),
                                                    p_AQ_lower=np.array(pose_list[i][1])-self.__constraint_tol,
                                                    p_AQ_upper=np.array(pose_list[i][1])+self.__constraint_tol)
        # add norminal state cost 
        # self.__IK.prog().AddQuadraticErrorCost(10*np.eye(self.__nq-7), q0[7:], self.__IK.q()[7:])

        self.__IK.prog().SetInitialGuess(self.__IK.q(), q0)

        snopt = SnoptSolver().solver_id()
        self.__IK.prog().SetSolverOption(snopt, 'Major Optimality Tolerance', self.__solver_tol)

        result = Solve(self.__IK.prog())
        if result.is_success():
            return [True, result.GetSolution(self.__IK.q())]
        return [False, []]


class CMIK:
    def __init__(self, plant, frame_name_list, constraint_tol=1.0e-8, solver_tol=1.0e-6):
        self.__plant = plant
        self.__plant_context = self.__plant.CreateDefaultContext()
        self.__ad_plant = self.__plant.ToAutoDiffXd()
        self.__ad_plant_context = self.__ad_plant.CreateDefaultContext()
        self.__nq = self.__plant.num_positions()
        self.__nv = self.__plant.num_velocities()
        self.__last_q = np.zeros(self.__nq)
        self.__constraint_tol = constraint_tol
        self.__solver_tol = solver_tol
        self.__frames = [self.__plant.GetFrameByName(name) for name in frame_name_list]

    def __CoMPositionConstraint(self, IK, r_des):
        com_position_constraint = ComPositionConstraint(self.__plant, None, self.__plant.world_frame(), self.__plant_context)
        r = IK.prog().NewContinuousVariables(3)  # r is the variable for CoM position.
        IK.prog().AddConstraint(com_position_constraint, np.concatenate([IK.q(), r]))
        b = IK.prog().AddBoundingBoxConstraint(r_des, r_des, r)
        IK.prog().SetInitialGuess(r, r_des)
        return b

    def __QVConstraint(self, dt, vars):
        q, v = np.split(vars, [self.__nq])
        qdot = self.__ad_plant.MapQDotToVelocity(self.__ad_plant_context, (q - self.__last_q))
        return (qdot - v)

    def __CMConstraint(self, IK, v, dt, k_WC_des, v0):
        k_WC_des = [v/float(dt) for v in k_WC_des]
        CM_constraint = CentroidalMomentumConstraint(plant=self.__ad_plant, model_instances=None,
                                                     plant_context=self.__ad_plant_context, angular_only=True)
        k_WC = IK.prog().NewContinuousVariables(3)
        IK.prog().AddConstraint(CM_constraint, np.concatenate([IK.q(), v, k_WC]))
        IK.prog().AddBoundingBoxConstraint(k_WC_des, k_WC_des, k_WC)
        IK.prog().SetInitialGuess(k_WC, k_WC_des)

        IK.prog().AddConstraint(partial(self.__QVConstraint, dt),
                                lb=np.zeros(self.__nv), ub=np.zeros(self.__nv),
                                vars=np.concatenate([IK.q(), v]))
        IK.prog().SetInitialGuess(v, np.zeros(self.__nv))

    def solve(self, pose_list, dt, q0=[], v0=[]):
        self.__last_q = q0
        self.__IK = InverseKinematics(plant=self.__plant, with_joint_limits=False)
        v = self.__IK.prog().NewContinuousVariables(self.__nv)
        for i, frame in enumerate(self.__frames):
            if not pose_list[i][0] is None:
                if i == 0:
                    self.__CMConstraint(self.__IK, v, dt, pose_list[i][0], v0)
                else:
                    self.__IK.AddOrientationConstraint(self.__plant.world_frame(), RotationMatrix(RollPitchYaw(pose_list[i][0])),
                                                       frame, RotationMatrix(RollPitchYaw(0, 0, 0)),
                                                       self.__constraint_tol)
            if not pose_list[i][1] is None:
                if i == 0:
                    self.__CoMPositionConstraint(self.__IK, pose_list[i][1])
                else:
                    self.__IK.AddPositionConstraint(frameB=frame,
                                                    p_BQ=np.zeros(3),
                                                    frameA=self.__plant.world_frame(),
                                                    p_AQ_lower=np.array(pose_list[i][1])-self.__constraint_tol,
                                                    p_AQ_upper=np.array(pose_list[i][1])+self.__constraint_tol)

        self.__IK.prog().SetInitialGuess(self.__IK.q(), q0)

        snopt = SnoptSolver().solver_id()
        self.__IK.prog().SetSolverOption(snopt, 'Major Optimality Tolerance', self.__solver_tol)

        result = Solve(self.__IK.prog())
        q, v = result.GetSolution(self.__IK.q()), result.GetSolution(v)
        v = [i/float(dt) for i in v]
        v = np.asarray(v)
        if result.is_success():
            return [True, q, v]
        return [False, [], []]


class velIK:
    def __init__(self, plant, frame_name_list, constraint_tol=1.0e-8, solver_tol=1.0e-6):
        self.__plant = plant
        self.__plant_context = self.__plant.CreateDefaultContext()
        self.__ad_plant = self.__plant.ToAutoDiffXd()
        self.__ad_plant_context = self.__ad_plant.CreateDefaultContext()
        self.__nq = self.__plant.num_positions()
        self.__nv = self.__plant.num_velocities()
        self.__last_q = np.zeros(self.__nq)
        self.__constraint_tol = constraint_tol
        self.__solver_tol = solver_tol
        self.__frames = [self.__plant.GetFrameByName(name) for name in frame_name_list]
        self.__frame_name_list = frame_name_list

    def _EndPointVelConstraint(self, vars, q0, endpoint_name):
        # ad_q0 = InitializeAutoDiff(q0, np.zeros((self.__nq, self.__nv))).squeeze()
        # self.__ad_plant.SetPositionsAndVelocities(self.__ad_plant_context, np.concatenate([ad_q0, vars]))
        # legvel = self.__ad_plant.GetFrameByName(endpoint_name).CalcSpatialVelocityInWorld(self.__ad_plant_context)
        # legvel = np.concatenate([legvel.rotational(), legvel.translational()])

        self.__plant.SetPositionsAndVelocities(self.__plant_context, np.concatenate([q0, ExtractValue(vars).squeeze()]))
        J_foot = self.__plant.CalcJacobianSpatialVelocity(
            self.__plant_context, JacobianWrtVariable.kV, self.__plant.GetFrameByName(endpoint_name),
            np.array([0,0,0]), self.__plant.world_frame(), self.__plant.world_frame())
        ad_dim = J_foot.shape[0]*J_foot.shape[1]
        ad_J_foot = InitializeAutoDiff(J_foot, np.zeros((ad_dim, self.__nv)))
        legvel = ad_J_foot.dot(vars)

        return legvel

    def __CMConstraint(self, IK, v, k_WC_des):
        CM_constraint = CentroidalMomentumConstraint(plant=self.__ad_plant, model_instances=None,
                                                     plant_context=self.__ad_plant_context, angular_only=False)
        k_WC = IK.prog().NewContinuousVariables(6)
        IK.prog().AddConstraint(CM_constraint, np.concatenate([IK.q(), v, k_WC]))
        IK.prog().AddBoundingBoxConstraint(k_WC_des, k_WC_des, k_WC)
        IK.prog().SetInitialGuess(k_WC, k_WC_des)


    def solve(self, pose_list, dt, q0, v0):
        self.__last_q = q0
        self.__IK = InverseKinematics(plant=self.__plant, with_joint_limits=False)

        self.__IK.prog().AddBoundingBoxConstraint(q0, q0, self.__IK.q())
        self.__IK.prog().SetInitialGuess(self.__IK.q(), q0)

        v = self.__IK.prog().NewContinuousVariables(self.__nv)
        self.__IK.prog().SetInitialGuess(v, v0)
        for i, endpoint_name in enumerate(self.__frame_name_list):
            if not pose_list[i][0] is None:
                if i == 0:
                    k_WC_des = np.concatenate([pose_list[i][0],pose_list[i][1]])
                    self.__CMConstraint(self.__IK, v, k_WC_des)
                else:
                    epvel_des = np.concatenate([pose_list[i][0],pose_list[i][1]])
                    self.__IK.prog().AddConstraint(partial(self._EndPointVelConstraint, q0=q0, endpoint_name=endpoint_name),
                                            lb=epvel_des, ub=epvel_des,
                                            vars = np.concatenate([v]))

        snopt = SnoptSolver().solver_id()
        self.__IK.prog().SetSolverOption(snopt, 'Major Optimality Tolerance', self.__solver_tol)

        result = Solve(self.__IK.prog())
        q_sol, v_sol = result.GetSolution(self.__IK.q()), result.GetSolution(v)

        self.__plant.SetPositions(self.__plant_context, q_sol)
        qdot_sol = self.__plant.MapVelocityToQDot(self.__plant_context, v_sol * dt)
        q_sol += qdot_sol
        q_sol[0:4] = q_sol[0:4]/np.linalg.norm(q_sol[0:4],ord=None,axis=None)


        if result.is_success():
            return [True, q_sol, v_sol]
        return [False, [], []]


if __name__ == "__main__":
    import time
    import argparse
    from pydrake.all import MultibodyPlant, FindResourceOrThrow, Parser, AddMultibodyPlantSceneGraph
    from pydrake.all import DiagramBuilder
    from pydrake.all import StartMeshcat, AddMultibodyPlantSceneGraph, AddDefaultVisualization

    # from pydrake.systems.meshcat_visualizer import ConnectMeshcatVisualizer
    from meshcat.servers.zmqserver import start_zmq_server_as_subprocess

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dt", type=float, default=-1,
                        help="Calc period.")
    args = parser.parse_args()

    try:
        time_step = 0 if args.dt < 0 else args.dt
        # time_step = 1e-3
        builder = DiagramBuilder()

        meshcat = StartMeshcat()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step)
        # filename = FindResourceOrThrow("drake/examples/atlas/urdf/atlas_minimal_contact.urdf")
        # filename = FindResourceOrThrow("~/humanoidjump/robots/atlas/urdf/atlas_minimal_contact.urdf")
        # filename = "~/humanoidjump/robots/atlas/urdf/atlas_minimal_contact.urdf"
        # filename = "robots/biped_v2/urdf/biped_v2_contact.urdf"
        filename = "robots/kuavo/urdf/New_new_kuavo_3_3.urdf"
        # filename = "robots/atlas/urdf/atlas_minimal_contact.urdf"
        parser = Parser(plant)
        parser.AddModelFromFile(filename)
        # Parser(plant).AddModels(filename)
        plant.Finalize()
        # proc, zmq_url, web_url = start_zmq_server_as_subprocess()
        # visualizer = ConnectMeshcatVisualizer(builder=builder,
        #                                       scene_graph=scene_graph,
        #                                       zmq_url=zmq_url,
        #                                       open_browser=False)
        
        AddDefaultVisualization(builder=builder, meshcat=meshcat)

        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        plant_context = plant.GetMyContextFromRoot(context)

        q0 = plant.GetPositions(plant_context)
        names = plant.GetPositionNames(False, False)
        state_names = plant.GetStateNames(False)
        print(q0.shape)
        print(q0)
        print((names))
        print(names)
        print(len(state_names))
        print(state_names)


        def init_state(plant, plant_context):
            foot_in_base = plant.GetFrameByName('l_foot_roll').CalcPose(plant_context, plant.GetFrameByName('base_link'))
            q0 = plant.GetPositions(plant_context)
            q0[6] = -foot_in_base.translation()[2]
            plant.SetPositions(plant_context, q0)

            l_arm_pitch = plant.GetJointByName(name="l_arm_pitch")
            r_arm_pitch = plant.GetJointByName(name="r_arm_pitch")
            l_arm_pitch.set_angle(plant_context, -1.57)
            r_arm_pitch.set_angle(plant_context,  1.57)

            l_arm_roll = plant.GetJointByName(name="l_arm_roll")
            r_arm_roll = plant.GetJointByName(name="r_arm_roll")
            l_arm_roll.set_angle(plant_context, -0.01)
            r_arm_roll.set_angle(plant_context,  0.01)

            l_arm_yaw = plant.GetJointByName(name="l_arm_yaw")
            r_arm_yaw = plant.GetJointByName(name="r_arm_yaw")
            l_arm_yaw.set_angle(plant_context, -0.001)
            r_arm_yaw.set_angle(plant_context,  0.001)

            l_forearm_pitch = plant.GetJointByName(name="l_forearm_pitch")
            r_forearm_pitch = plant.GetJointByName(name="r_forearm_pitch")
            l_forearm_pitch.set_angle(plant_context, -0.1)
            r_forearm_pitch.set_angle(plant_context,  0.1)

            l_forearm_yaw = plant.GetJointByName(name="l_forearm_yaw")
            r_forearm_yaw = plant.GetJointByName(name="r_forearm_yaw")
            l_forearm_yaw.set_angle(plant_context, -0.001)
            r_forearm_yaw.set_angle(plant_context,  0.001)

            l_hand_roll = plant.GetJointByName(name="l_hand_roll")
            r_hand_roll = plant.GetJointByName(name="r_hand_roll")
            l_hand_roll.set_angle(plant_context, -0.001)
            r_hand_roll.set_angle(plant_context,  0.001)

            l_hand_pitch = plant.GetJointByName(name="l_hand_pitch")
            r_hand_pitch = plant.GetJointByName(name="r_hand_pitch")
            l_hand_pitch.set_angle(plant_context, -0.001)
            r_hand_pitch.set_angle(plant_context,  0.001)

            l_leg_roll = plant.GetJointByName(name="l_leg_roll")
            r_leg_roll = plant.GetJointByName(name="r_leg_roll")
            l_leg_roll.set_angle(plant_context, -0.01561)
            r_leg_roll.set_angle(plant_context,  0.01507)

            l_leg_yaw = plant.GetJointByName(name="l_leg_yaw")
            r_leg_yaw = plant.GetJointByName(name="r_leg_yaw")
            l_leg_yaw.set_angle(plant_context, -0.00164)
            r_leg_yaw.set_angle(plant_context,  0.00158)

            l_leg_pitch = plant.GetJointByName(name="l_leg_pitch")
            r_leg_pitch = plant.GetJointByName(name="r_leg_pitch")
            l_leg_pitch.set_angle(plant_context, 0.64207)
            r_leg_pitch.set_angle(plant_context,  0.64204)

            l_knee = plant.GetJointByName(name="l_knee")
            r_knee = plant.GetJointByName(name="r_knee")
            l_knee.set_angle(plant_context,  0.88718)
            r_knee.set_angle(plant_context, -0.88714)

            l_foot_pitch = plant.GetJointByName(name="l_foot_pitch")
            r_foot_pitch = plant.GetJointByName(name="r_foot_pitch")
            l_foot_pitch.set_angle(plant_context,  0.34983)
            r_foot_pitch.set_angle(plant_context,  0.34981)

            l_foot_roll = plant.GetJointByName(name="l_foot_roll")
            r_foot_roll = plant.GetJointByName(name="r_foot_roll")
            l_foot_roll.set_angle(plant_context,  0.01570)
            r_foot_roll.set_angle(plant_context, -0.01515)

            l_hand_in_base = plant.GetFrameByName('l_hand_pitch').CalcPose(plant_context, plant.GetFrameByName('base_link'))
            print("left hand position in base:", l_hand_in_base.translation())

            l_hand_in_foot = plant.GetFrameByName('l_hand_pitch').CalcPose(plant_context, plant.GetFrameByName('l_foot_roll'))
            print("left hand position in foot:", l_hand_in_foot.translation())

            r_hand_in_base = plant.GetFrameByName('r_hand_pitch').CalcPose(plant_context, plant.GetFrameByName('base_link'))
            print("right hand position in base:", r_hand_in_base.translation())

            r_hand_in_foot = plant.GetFrameByName('r_hand_pitch').CalcPose(plant_context, plant.GetFrameByName('r_foot_roll'))
            print("right hand position in foot:", r_hand_in_foot.translation())

            print("foot_in_base: ", foot_in_base)
            p_left_hand = l_hand_in_base.translation()
            p_right_hand = r_hand_in_base.translation()

            q0 = plant.GetPositions(plant_context)
            
            print(p_left_hand)
            # p_left_hand[0] = -p_left_hand[0]

            r0 = plant.CalcCenterOfMassPositionInWorld(plant_context)
            squat = 0.06
            # frame_list = ['base_link', 'l_foot_roll', 'r_foot_roll']#, 'r_hand_pitch']
            # frame_list = ['base_link','l_foot_roll', 'r_foot_roll']#, 'r_hand_pitch']
            frame_list = ['base_link','l_hand_pitch','r_hand_pitch','l_foot_roll', 'r_foot_roll']
            IK = CoMIK(plant, frame_list)
            pose_list = [
                [[0.0, 0.0, 0.0], [0, 0, r0[2]]],
                [None, p_left_hand],
                [None, p_right_hand],
                # [None, [p_left_hand[0], p_left_hand[1], p_left_hand[2]-0.1]],
                # [None, [p_right_hand[0], p_right_hand[1], p_right_hand[2]-0.1]],
                [[0.0, 0.0, 0.0], [0.0, foot_in_base.translation()[1], 0.0]],
                [[0.0, 0.0, 0.0], [0.0, -foot_in_base.translation()[1], 0.0]]
            ]
            is_success, q = IK.solve(pose_list, q0=q0)
            if not is_success:
                print('Failed to IK0!')
                exit()
            plant.SetPositions(plant_context, q)

        init_state(plant, plant_context)
        q0 = plant.GetPositions(plant_context)
        v0 = plant.GetVelocities(plant_context)
        r0 = plant.CalcCenterOfMassPositionInWorld(plant_context)

        # foot_pos = plant.GetFrameByName('l_foot_sole').CalcPose(plant_context, plant.world_frame()).translation()
        # frame_list = ['base_link', 'l_hand_pitch', 'r_hand_pitch', 'l_foot_roll', 'r_foot_roll']
        # IK = CMIK(plant, frame_list)

        # # visualizer.load()
        # # visualizer.start_recording()
        # # visualizer.publish_recording()
        # # if args.dt < 0:
        # #     args.dt = visualizer.draw_period
        # print("dt: {}".format(args.dt))

        # last_q = q0
        # last_v = v0
        # A, T, b = 0.1, 2, 0
        # t = 0
        # t_total = T * 2
        # while True:
        #     t0 = time.time()

        #     traj = A * -np.cos(2 * np.pi / T * t) + b + A
        #     pose_list = [
        #         [[0.0, 0.0, 0.0], [0.0, 0.0, r0[2] - traj]],
        #         [[0.0, 0.0, 0.0], [0.0, foot_pos[1], 0.0]],
        #         [[0.0, 0.0, 0.0], [0.0, -foot_pos[1], 0.0]],
        #         [[0.0, 0.0, 0.0], None]
        #     ]
        #     is_success, q, v = IK.solve(pose_list, args.dt, q0=last_q, v0=last_v)
        #     if is_success:
        #         plant.SetPositions(plant_context, q)
        #         last_q = q
        #         last_v = v
        #     else:
        #         print('Failed to IK!')
        #     context.SetTime(t)
        #     diagram.Publish(context)

        #     t += args.dt
        #     print('{:.4f}/{:.4f}'.format(t, t_total), end='\r')
        #     if t >= t_total:
        #         print('{:.4f}/{:.4f}'.format(t, t_total), end='\n')
        #         break

        #     t_sleep = args.dt-(time.time()-t0)
        #     time.sleep(t_sleep if t_sleep > 0 else 0)

        # visualizer.stop_recording()
        # visualizer.publish_recording()
        diagram.ForcedPublish(context)

        print('Program end, Press Ctrl + C to exit.')
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("")

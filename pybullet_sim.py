import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import gait_controller as gait
import kinematics
from utils import *

pi = math.pi

# --- CONFIGURATION ---
urdf_path = "./urdf/spotmicroai_gen_ros.urdf"  
theta_dirs = [-1, 1, 1,
                1, 1, 1,
                -1, 1, 1, 
                1, 1, 1]
joint_dic = {"front_left_shoulder":3,
                "front_left_leg":4,
                "front_left_foot": 6,
                "front_right_shoulder" : 8,
                "front_right_leg": 9,
                "front_right_foot": 11,
                "rear_left_shoulder": 13,
                "rear_left_leg": 14,
                "rear_left_foot": 16,
                "rear_right_shoulder": 18,
                "rear_right_leg": 19,
                "rear_right_foot": 21}

def move_robot_to_pose(robotId, theta, joint_dic):
    
    # Convert angles to radians and apply directions
    theta = [math.radians(angle) * dir for angle, dir in zip(theta, theta_dirs)]
    p.setJointMotorControlArray(robotId,
                                jointIndices=list(joint_dic.values()),
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=theta)
    

def display_robot_state(center_plane, center, orientation, theta, l, w, l1, l2, l3, l4):
    kinematics_solver = kinematics.Kinematics(l, w, l1, l2, l3, l4)
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=-1.6, cameraPitch=-165, cameraTargetPosition=[0, 0, 0])

    orn= p.getQuaternionFromEuler(orientation)  # Roll, Pitch, Yaw in radians

    planeId = p.loadURDF("plane.urdf", center_plane, orn, useFixedBase=True)
    
    try:
        
        robotId = p.loadURDF(urdf_path, center, orn, useFixedBase=True)
        print(f"Successfully loaded {urdf_path}!")
        num_joints = p.getNumJoints(robotId)
        print(f"Robot has {num_joints} joints.")
        for i in range(num_joints):
            info = p.getJointInfo(robotId, i)
            joint_name = info[1].decode("utf-8")
            joint_type = info[2]            
            # We only care about movable joints (Revolute or Prismatic)
            if joint_type == p.JOINT_REVOLUTE or joint_type == p.JOINT_PRISMATIC:
                print(f"Loaded Joint: {joint_name} (ID: {i})")
    except Exception as e:
        print(f"Error loading URDF: {e}")
        return

    # Starting Pose
    move_robot_to_pose(robotId, theta, joint_dic)
    center_kin = from_pybullet(center)
    while True:
        p.stepSimulation()

        # x+30, y+20, z-20
        pointp = [ 97.5, 26.48,  87]
        pointp = to_pybullet(pointp)

        point_x = np.array([1, 0, 0])
        point_y = np.array([0, 1, 0])
        point_z = np.array([0, 0, 1])

        #### 15 x z
        eof_positions = kinematics_solver.robot_FK(center_kin, [0, 0, 0], theta, unit='degrees')
        
        eof_positions_pb = np.array([to_pybullet(pos[:3]) for pos in eof_positions])
        

        fl_eof = eof_positions_pb[0]
        fl_eof_end = [fl_eof[0]+0.1, fl_eof[1], fl_eof[2]]
        # print(f"Front Left EOF PyBullet:{fl_eof}")
        # print(f"Front Left EOF Expected:{pointp}")
        fr_eof = eof_positions_pb[1]
        rl_eof = eof_positions_pb[2]
        rr_eof = eof_positions_pb[3]

        

        visual_idx = p.createVisualShape(p.GEOM_SPHERE, radius=0.01, rgbaColor=[1, 0, 0, 1])
        
        p.createMultiBody(baseVisualShapeIndex=visual_idx, basePosition=fl_eof)
        p.createMultiBody(baseVisualShapeIndex=visual_idx, basePosition=fl_eof_end)
        # p.createMultiBody(baseVisualShapeIndex=visual_idx, basePosition=fr_eof)
        # p.createMultiBody(baseVisualShapeIndex=visual_idx, basePosition=rl_eof)
        # p.createMultiBody(baseVisualShapeIndex=visual_idx, basePosition=rr_eof)

        # visual_idy = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 1, 0, 1])
        # p.createMultiBody(baseVisualShapeIndex=visual_idy, basePosition=point_y)

        # visual_idz = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 0, 1, 1])
        # p.createMultiBody(baseVisualShapeIndex=visual_idz, basePosition=point_z)
       
        try:

            # Testing only front left leg
            mouse_event = p.getMouseEvents()
            event_type = mouse_event[0][0]
            button_state = mouse_event[0][-1]
            if event_type == 2 and button_state == 4:  # Mouse left button released
                
                print("Start Pos:", fl_eof)
                print("End Pos:", fl_eof_end)

                gait_controller = gait.GaitController()

                target_angles, curve_points, control_points = gait_controller.swing_trajectory(
                    start_pos=fl_eof,  
                    end_pos=fl_eof_end,  
                    swing_height=0.07,            
                    center=center,
                    orientation=[0, 0, 0],
                    phase=0
                )
                for target_angle in target_angles:

                    p.setJointMotorControlArray(robotId,
                                    jointIndices=[3, 4, 6],  # Front Left ankle, knee, hip
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=target_angle)
                    for _ in range(3): 
                        p.stepSimulation()
                        time.sleep(1./240.)

                # for cp in curve_points:
                    
                #     p_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.001, rgbaColor=[0, 1, 0, 1])
                #     p.createMultiBody(baseVisualShapeIndex=p_visual, basePosition=cp)

                

                
                
                

                

                
                

        except:
            pass
        time.sleep(1./240.) # PyBullet default time step

if __name__ == "__main__":

    start_pos = [0, 0, 0.25]    
    start_pos_plane = [0, 0, 0] 
    # X forward Y up Z left
    start_orn = [0, 0, pi]  # Roll, Pitch, Yaw in radians

    theta = [0, -30, 60, # FL
             0, -30, 60, # FR
             0, -30, 60, # RL
             0, -30, 60 ] # RR
    
    eof_positions = np.array([
            [ 95, 48.13,  105, 1], # FL
            [ 95, 48.13,  -105, 1], # FR
            [-45, 48.13, 105, 1], # RL
            [-45, 48.13, -105, 1] # RR
            ])
    


    
    


    display_robot_state(center_plane=start_pos_plane, 
                        center=start_pos, 
                        orientation=start_orn, 
                        theta=theta, 
                        l=kinematics.LENGTH, 
                        w=kinematics.WIDTH,
                        l1=kinematics.L1, 
                        l2=kinematics.L2, 
                        l3=kinematics.L3, 
                        l4=kinematics.L4)
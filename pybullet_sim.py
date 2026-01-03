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




def display_robot_state(center_plane, center, orientation, theta, w, l, l1, l2, l3, l4):
    kinematics_solver = kinematics.Kinematics(w, l, l1, l2, l3, l4)
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=-1.6, cameraPitch=-165, cameraTargetPosition=[0, 0, 0])

    orn= p.getQuaternionFromEuler(orientation)  # Roll, Pitch, Yaw in radians

    planeId = p.loadURDF("plane.urdf", center_plane, orn, useFixedBase=True)
    
    try:
        
        robotId = p.loadURDF(urdf_path, center, orn, useFixedBase=True)
        print(f"Successfully loaded {urdf_path}!")
    except Exception as e:
        print(f"Error loading URDF: {e}")
        return

    num_joints = p.getNumJoints(robotId)
    joint_ids = []
    param_ids = []
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
    
    print(f"Robot has {num_joints} joints.")
    
    for i in range(num_joints):
        info = p.getJointInfo(robotId, i)
        joint_name = info[1].decode("utf-8")
        joint_type = info[2]
        
        # We only care about movable joints (Revolute or Prismatic)
        if joint_type == p.JOINT_REVOLUTE or joint_type == p.JOINT_PRISMATIC:
            joint_ids.append(i)
            print(f"Loaded Joint: {joint_name} (ID: {i})")
            angle_i = list(joint_dic.values()).index(i)
            step_angles = np.deg2rad(theta[angle_i])
            p.setJointMotorControl2(
                bodyIndex=robotId,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=step_angles
            )
    
    center_kin = from_pybullet(center)
    while True:
        p.stepSimulation()

        pointp = [ 95, 48.13,  105]
        pointp = to_pybullet(pointp)

        point_x = np.array([1, 0, 0])
        point_y = np.array([0, 1, 0])
        point_z = np.array([0, 0, 1])

        
        eof_positions = kinematics_solver.robot_FK(center_kin, [0, 0, 0], theta, unit='degrees')
        
        eof_positions_pb = np.array([to_pybullet(pos[:3]) for pos in eof_positions])

        fl_eof = eof_positions_pb[0]
        fr_eof = eof_positions_pb[1]
        rl_eof = eof_positions_pb[2]
        rr_eof = eof_positions_pb[3]

        

        visual_idx = p.createVisualShape(p.GEOM_SPHERE, radius=0.01, rgbaColor=[1, 0, 0, 1])
        p.createMultiBody(baseVisualShapeIndex=visual_idx, basePosition=fl_eof)
        p.createMultiBody(baseVisualShapeIndex=visual_idx, basePosition=fr_eof)
        p.createMultiBody(baseVisualShapeIndex=visual_idx, basePosition=rl_eof)
        p.createMultiBody(baseVisualShapeIndex=visual_idx, basePosition=rr_eof)

        # visual_idy = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 1, 0, 1])
        # p.createMultiBody(baseVisualShapeIndex=visual_idy, basePosition=point_y)

        # visual_idz = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 0, 1, 1])
        # p.createMultiBody(baseVisualShapeIndex=visual_idz, basePosition=point_z)
       
        try:

            # Testing only front left leg
            mouse_event = list(p.getMouseEvents())[0][0]
            if mouse_event == 2:
                
                print("Start Pos:", point_x)
                print("End Pos:", point_y)
                

                
                
                

                

                
                

        except:
            pass
        time.sleep(1./240.) # PyBullet default time step

if __name__ == "__main__":

    start_pos = [0, 0, 0.25]    
    start_pos_plane = [0, 0, 0] 
    # X forward Y up Z left
    start_orn = [0, 0, pi]  # Roll, Pitch, Yaw in radians

    theta = [0, -30, 60,  # FL
             0, -30, 60,  # FR
             0, -30, 60,  # RL
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
                        w=kinematics.WIDTH, 
                        l=kinematics.LENGTH,
                        l1=kinematics.L1, 
                        l2=kinematics.L2, 
                        l3=kinematics.L3, 
                        l4=kinematics.L4)
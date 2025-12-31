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
start_pos = [0, 0, 0.25]    
start_pos_plane = [0, 0, 0] 
# X forward Y up Z left
start_orn = p.getQuaternionFromEuler([0, 0, pi])  



def display_robot_state(theta):
    kinematics_solver = kinematics.Kinematics(kinematics.L1, kinematics.L2, kinematics.L3)
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=125, cameraPitch=-35, cameraTargetPosition=[0, 0, 0])
    planeId = p.loadURDF("plane.urdf", start_pos_plane, start_orn, useFixedBase=True)
    
    try:
        robotId = p.loadURDF(urdf_path, start_pos, start_orn, useFixedBase=True)
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
    while True:
        p.stepSimulation()
        
        point_x = np.array([95 /1000, 105 /1000, 48 /1000])
        point_y = np.array([170 /1000, 105 /1000, 48 /1000])
        point_z = np.array([0, 0, 1])

        visual_idx = p.createVisualShape(p.GEOM_SPHERE, radius=0.012, rgbaColor=[1, 0, 0, 1])
        point_idx = p.createMultiBody(baseVisualShapeIndex=visual_idx, basePosition=point_x)

        visual_idy = p.createVisualShape(p.GEOM_SPHERE, radius=0.01, rgbaColor=[0, 1, 0, 1])
        point_idy = p.createMultiBody(baseVisualShapeIndex=visual_idy, basePosition=point_y)

        visual_idz = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 0, 1, 1])
        point_idz = p.createMultiBody(baseVisualShapeIndex=visual_idz, basePosition=point_z)
       
        try:
            mouse_event = list(p.getMouseEvents())[0][0]
            if mouse_event == 2:
                gait_controller = gait.GaitController(stance_time=0.5, 
                                                      swing_time=0.5, 
                                                      time_step=1./240., 
                                                      contact_phases=None, 
                                                      default_stance=None)
                print("Start Pos:", point_x)
                print("End Pos:", point_y)


                

                

                trajectory, _, _ = gait_controller.swing_trajectory(
                    start_pos=point_x,
                    end_pos=point_y,
                    swing_height=0.1,
                    phase=0
                )

                angles = trajectory
                
                for step_angles in angles:
                    # add a small delay to visualize the movement
                    p.setJointMotorControlArray(
                        bodyIndex=robotId,
                        jointIndices=[6, 4, 3],
                        controlMode=p.POSITION_CONTROL,
                        targetPositions=step_angles
                            )

                
                pass

        except:
            pass
        time.sleep(1./240.) # PyBullet default time step

if __name__ == "__main__":

    theta = [0, -30, 60,  # FL
             0, -30, 60,  # FR
             0, -30, 60,  # RL
             0, -30, 60 ] # RR
    
    eof_positions = np.array([
        [ 100, -200,  100, 1], # FL
        [ 150, -200, -100, 1], # FR
        [-100, -200,  100, 1], # RL
        [-100, -200, -100, 1]  # RR
        ])
    
    


    display_robot_state(theta)
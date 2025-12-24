import pybullet as p
import pybullet_data
import time
import math
import numpy as np

pi = math.pi

# --- CONFIGURATION ---
urdf_path = "./urdf/spotmicroai_gen_ros.urdf"  
start_pos = [0, 0, 0.5]     
start_orn = p.getQuaternionFromEuler([0, 0, 0])

def display_robot_state(theta):
    
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=-45, cameraPitch=-35, cameraTargetPosition=[0, 0, 0])
    planeId = p.loadURDF("plane.urdf")
    
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
            angle = np.deg2rad(theta[angle_i])
            p.setJointMotorControl2(
                bodyIndex=robotId,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=angle
            )
    angle = list(p.getJointState(robotId, 4))[0]
    print(angle)
    while True:
        p.stepSimulation()
        
        try:
            mouse_event = list(p.getMouseEvents())[0][0]
            if mouse_event == 2:
                print(angle)
                angle += 0.01
                p.setJointMotorControl2(
                bodyIndex=robotId,
                jointIndex=4,
                controlMode=p.POSITION_CONTROL,
                targetPosition=angle
            )
        except:
            pass
        # if mouse_event[0] == 2:
        #     print("ok")
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
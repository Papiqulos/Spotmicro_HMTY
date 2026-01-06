import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import gait_controller as gait
import kinematics
from utils import from_pybullet, to_pybullet


# CONSTANTS
PI = math.pi

class PybulletSim:
    
    def __init__(self, length, width, l1, l2, l3, l4, center, orientation, center_plane, initial_theta):
        # --- CONFIGURATION ---
        self.urdf_path = "./urdf/spotmicroai_gen_ros.urdf"  
        self.theta_dirs = [-1, 1, 1,
                        1, 1, 1,
                        -1, 1, 1, 
                        1, 1, 1]
        self.joint_dic = {"front_left_shoulder":3,
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
        # Robot Parameters
        self.length = length
        self.width = width
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
        # Kinematics and Gait Controllers
        self.kin_solver = kinematics.Kinematics(self.length, self.width, self.l1, self.l2, self.l3, self.l4)
        self.gait_solver = gait.GaitController()

        # Pybullet Setup
        self.prep_environment(plane_orientation=[0, 0, 0], center_plane=center_plane)
        self.robotId, self.num_joints = self.load_quadruped(self.urdf_path, center, p.getQuaternionFromEuler(orientation))

        # Display Initial Pose and Start Simulation
        self.display_initial_pose(center, initial_theta)

    def load_quadruped(self, urdf_path, center, orn):
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
            return robotId, num_joints
        except Exception as e:
            print(f"Error loading URDF: {e}")
            return
        
    def prep_environment(self, plane_orientation, center_plane,cameraDistance=1, cameraYaw=-181, cameraPitch=-165, cameraTargetPosition=[0, 0, 0]):
        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.resetDebugVisualizerCamera(cameraDistance=cameraDistance, cameraYaw=cameraYaw, cameraPitch=cameraPitch, cameraTargetPosition=cameraTargetPosition)

        orn= p.getQuaternionFromEuler(plane_orientation)  # Roll, Pitch, Yaw in radians
        planeId = p.loadURDF("plane.urdf", center_plane, orn, useFixedBase=True)

    def move_robot_to_pose(self, robotId, theta, joint_dic, unit='degrees'):
        if unit == 'degrees':
            theta = [math.radians(angle) for angle in theta]
        # Convert angles to radians and apply directions
        theta = [angle * dir for angle, dir in zip(theta, self.theta_dirs)]
        p.setJointMotorControlArray(robotId,
                                    jointIndices=list(joint_dic.values()),
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=theta)
    
    def execute_leg_trajectory(self, trajectory, leg="FL"):
        """ Execute a trajectory for a single leg.
        """
        if leg == "FL":
            jointIndices = [3, 4, 6]
            dirs = self.theta_dirs[:3]
        elif leg == "FR":
            jointIndices = [8, 9, 11]
            dirs = self.theta_dirs[3:6]
        elif leg == "RL":
            jointIndices = [13, 14, 16]
            dirs = self.theta_dirs[6:9]
        elif leg == "RR":
            jointIndices = [18, 19, 21]
            dirs = self.theta_dirs[9:12]
        
        for target_angle in trajectory:
            # Apply theta dirs
            target_angle = [angle * dir for angle, dir in zip(target_angle, dirs)]
            p.setJointMotorControlArray(self.robotId,
                            jointIndices=jointIndices,  # shoulder, leg, foot
                            controlMode=p.POSITION_CONTROL,
                            targetPositions=target_angle)
            for _ in range(2): 
                p.stepSimulation()
                time.sleep(1./120.)

    def display_initial_pose(self, center, theta):
        
        # Initial Pose
        self.move_robot_to_pose(self.robotId, theta, self.joint_dic)
        center_kin = from_pybullet(center)
        # Start Simulation Loop
        while True:
            p.stepSimulation()

            # X forward Y left Z up
            # x+30, y+20, z-20
            #### 15 x z
            eof_positions = self.kin_solver.robot_FK(center_kin, [0, 0, 0], theta, unit='degrees')
            eof_positions_pb = np.array([to_pybullet(pos[:3]) for pos in eof_positions])

            # for point in eof_positions_pb:
            #     self.debug_point(point)

            fl_eof = eof_positions_pb[0]
            fr_eof = eof_positions_pb[1]
            rl_eof = eof_positions_pb[2]
            rr_eof = eof_positions_pb[3]

            
            mouse_event = p.getMouseEvents()
            try:
                event_type = mouse_event[0][0]
                button_state = mouse_event[0][-1]
            except:
                event_type = None
                button_state = None
            # Mouse left button released
            if event_type == 2 and button_state == 4:  
                leg = "FR"

                if leg == "FL":
                    start_pos = fl_eof
                elif leg == "FR":
                    start_pos = fr_eof
                elif leg == "RL":
                    start_pos = rl_eof
                elif leg == "RR":
                    start_pos = rr_eof
                
                # Swing
                swing_angles, _, _ = self.gait_solver.swing_trajectory(
                    start_pos=start_pos,    
                    swing_height=0.1,            
                    center=center,
                    orientation=[0, 0, 0],
                    leg=leg,
                    disp_length=0.08,
                    disp_orientation="+x"
                )

                # Stance
                stance_angles, _, _ = self.gait_solver.stance_trajectory(
                    start_pos=start_pos,
                    center=center,
                    orientation=[0, 0, 0],
                    leg=leg,
                    disp_length=0.08,
                    disp_orientation="-x"
                )
                
                self.execute_leg_trajectory(swing_angles, leg=leg)
                # self.execute_leg_trajectory(stance_angles, leg=leg)
                

                
            time.sleep(1./240.) # PyBullet default time step

    def debug_point(self, point, colour=[1, 0, 0, 1], radius=0.01):
        visual_idx = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=colour)
        p.createMultiBody(baseVisualShapeIndex=visual_idx, basePosition=point)

if __name__ == "__main__":
    # X forward Y up Z left

    center = [0, 0, 0.3]    
    center_plane = [0, 0, 0] 
    orientation = [0, 0, PI]  # Roll, Pitch, Yaw in radians

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


    pybullet_sim = PybulletSim(length=kinematics.LENGTH, 
                               width=kinematics.WIDTH,
                               l1=kinematics.L1, 
                               l2=kinematics.L2, 
                               l3=kinematics.L3, 
                               l4=kinematics.L4,
                               center=center,
                               orientation=orientation,
                               center_plane=center_plane,
                               initial_theta=theta)
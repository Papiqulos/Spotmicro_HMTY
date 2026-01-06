import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import gait_controller as gait
import kinematics
from utils import from_pybullet_pos, to_pybullet_pos, from_pybullet_orn, to_pybullet_orn



# CONSTANTS
PI = math.pi

class PybulletSim:
    
    def __init__(self, 
                 length, 
                 width, 
                 l1, 
                 l2, 
                 l3, 
                 l4, 
                 center, 
                 orientation, 
                 center_plane, 
                 initial_theta, 
                 initial_ef_positions, 
                 angle_unit='degrees'):
        """
        
        :param length: robot base length in mm
        :param width: robot base width in mm
        :param l1: 
        :param l2: 
        :param l3: 
        :param l4: 
        :param center: initial center for the robot to spawn (in pybullet frame)
        :param orientation: initial orientation for the robot to spawn (in pybullet frame)
        :param center_plane: in pybullet frame
        :param initial_theta: initial angles for the robot to spawn
        :param initial_ef_positions: in kinematics frame and homogenous coordinates
        :param angle_unit: unit of initial_theta
        """
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
        self.center = center
        self.center_kin = from_pybullet_pos(center)
        self.orientation = orientation
        self.orientation_kin = from_pybullet_orn(orientation)
        self.center_plane = center_plane
        self.initial_theta = initial_theta
        self.initial_ef_positions = initial_ef_positions
        self.angle_unit = angle_unit

        # Kinematics and Gait Controllers
        self.kin_solver = kinematics.Kinematics(self.length, 
                                                self.width, 
                                                self.l1, 
                                                self.l2, 
                                                self.l3, 
                                                self.l4)
        self.gait_solver = gait.GaitController(initial_ef_positions=initial_ef_positions, 
                                               initial_theta=initial_theta, 
                                               initial_center=self.center_kin, 
                                               initial_orientation=self.orientation_kin)

        # Pybullet Setup
        # Prepare environment
        self.prep_environment(plane_orientation=[0, 0, 0], center_plane=center_plane)
        # Load the quadruped
        self.robotId, self.num_joints = self.load_quadruped(self.urdf_path, center, p.getQuaternionFromEuler(orientation))

        # Display Initial Pose and Start Simulation
        self.display_initial_pose(center, initial_theta)

    def load_quadruped(self, urdf_path, center, orn):
        """
        Load the urdf with its meshes
        
        :param urdf_path: file path
        :param center: in pybullet frame
        :param orn: in pybullet frame
        """
        try:
            robotId = p.loadURDF(urdf_path, center, orn, )
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
        """
        Docstring for prep_environment
        
        :param plane_orientation: in pybullet frame
        :param center_plane: in pybullet frame
        :param cameraDistance: 
        :param cameraYaw: 
        :param cameraPitch: 
        :param cameraTargetPosition: 
        """
        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.resetDebugVisualizerCamera(cameraDistance=cameraDistance, cameraYaw=cameraYaw, cameraPitch=cameraPitch, cameraTargetPosition=cameraTargetPosition)

        orn= p.getQuaternionFromEuler(plane_orientation)  # Roll, Pitch, Yaw in radians
        planeId = p.loadURDF("plane.urdf", center_plane, orn, useFixedBase=True)

    def move_robot_to_pose(self, robotId, theta, joint_dic, unit='degrees'):
        """
        Move the robot to a given pose.
        
        :param robotId: 
        :param theta: angles for all legs [[FL], [FR], [RL], [RR]]
        :param joint_dic: dictionary mapping joint names to joint indices
        :param unit: angle unit
        """

        if unit == 'degrees':
            theta = [math.radians(angle) for angle in theta]
        # Convert angles to radians and apply directions
        theta = [angle * dir for angle, dir in zip(theta, self.theta_dirs)]
        p.setJointMotorControlArray(robotId,
                                    jointIndices=list(joint_dic.values()),
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=theta)
    
    def execute_leg_trajectory(self, trajectory, leg="FL"):
        """ 
        Execute a trajectory for a single leg.

        :param trajectory: trajectory of angles for the leg
        :param leg: leg to execute the trajectory for

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
        
        """
        Display the initial and run some gait tests
        
        :param center: in pybullet frame
        :param theta: initial angles for all legs [[FL], [FR], [RL], [RR]]. Unit is given in self.angle_unit
        """

        # Initial Pose
        self.move_robot_to_pose(self.robotId, theta, self.joint_dic, self.angle_unit)
        
        
        # Start Simulation Loop
        while True:
            p.stepSimulation()

            # X forward Y left Z up
            # x+30, y+20, z-20
            #### 15 x z
            ef_positions = self.kin_solver.robot_FK(center_kin, [0, 0, 0], theta, unit=self.angle_unit)
            ef_positions_pb = np.array([to_pybullet_pos(pos[:3]) for pos in ef_positions])

            # for point in ef_positions_pb:
            #     self.debug_point(point)

            fl_ef = ef_positions_pb[0]
            fr_ef = ef_positions_pb[1]
            rl_ef = ef_positions_pb[2]
            rr_ef = ef_positions_pb[3]

            
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
                    start_pos = fl_ef
                elif leg == "FR":
                    start_pos = fr_ef
                elif leg == "RL":
                    start_pos = rl_ef
                elif leg == "RR":
                    start_pos = rr_ef
                
                # Swing
                swing_angles, _, _ = self.gait_solver.swing_trajectory(
                    start_pos=start_pos,    
                    swing_height=0.1,            
                    leg=leg,
                    stride_len=0.08,
                    stride_orn="+x"
                )

                # Stance
                stance_angles, _, _ = self.gait_solver.stance_trajectory(
                    start_pos=start_pos,
                    leg=leg,
                    stride_len=0.08,
                    stride_orn="-x"
                )
                
                self.execute_leg_trajectory(swing_angles, leg=leg)
                # self.execute_leg_trajectory(stance_angles, leg=leg)
                

                
            time.sleep(1./240.) # PyBullet default time step

    def debug_point(self, point, colour=[1, 0, 0, 1], radius=0.01):
        """
        Generate a point in pybullet
        
        :param point: coordinates in pybullet frame
        :param colour: colour in rgba format
        :param radius: radius in meters
        """
        visual_idx = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=colour)
        p.createMultiBody(baseVisualShapeIndex=visual_idx, basePosition=point)

if __name__ == "__main__":

    # X forward Y up Z left in meters
    center = [0, 0, 0.25]    
    center_plane = [0, 0, 0] 
    # This is the default orientation of the pybullet frame which is equivalent to [0, 0, 0] in the kinematics frame
    orientation = [0, 0, PI]  # Roll, Pitch, Yaw in radians

    theta = [0, -30, 60, # FL
             0, -30, 60, # FR
             0, -30, 60, # RL
             0, -30, 60 ] # RR
    
    ef_positions = np.array([
            [ 95, 48.13,  105, 1], # FL
            [ 95, 48.13,  -105, 1], # FR
            [-45, 48.13, 105, 1], # RL
            [-45, 48.13, -105, 1] # RR
            ])
    
    ef_positions2 = np.array([
        [67.29, 46.12, 107, 1],
        [67.29, 46.12, -107, 1],
        [-72.21, 46.12, 107, 1],
        [-72.21, 46.12, -107, 1]
        ])
    
    kin = kinematics.Kinematics(kinematics.LENGTH, 
                                kinematics.WIDTH, 
                                kinematics.L1, 
                                kinematics.L2, 
                                kinematics.L3, 
                                kinematics.L4)

    center_kin = from_pybullet_pos(center)
    orientation_kin = from_pybullet_orn(orientation)
    print(orientation_kin)

    angles = kin.robot_IK(center_kin, orientation_kin, ef_positions2)

    pybullet_sim = PybulletSim(length=kinematics.LENGTH, 
                               width=kinematics.WIDTH,
                               l1=kinematics.L1, 
                               l2=kinematics.L2, 
                               l3=kinematics.L3, 
                               l4=kinematics.L4,
                               center=center,
                               orientation=orientation,
                               center_plane=center_plane,
                               initial_theta=angles,
                               initial_ef_positions=ef_positions,
                               angle_unit="radians")
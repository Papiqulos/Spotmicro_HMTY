import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import core.gait_controller as gait
import core.kinematics as kinematics_old
from tools.utils import from_pybullet_orn, from_pybullet_pos
from collections import deque



# CONSTANTS
PI = math.pi
TIME_STEP = 1. / 240.

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
        :param initial_ef_positions: initial end effector positions for the robot to spawn (in kinematics frame)
        :param angle_unit: unit of initial_theta
        """
        # --- CONFIGURATION ---
        self.urdf_path = "sim/urdf/spotmicroai_gen_ros.urdf"  
        self.theta_dirs = [-1, 1, 1,
                        1, 1, 1,
                        -1, 1, 1, 
                        1, 1, 1]
        # self.joint_dic = {"front_left_shoulder":3,
        #             "front_left_leg":4,
        #             "front_left_foot": 6,
        #             "front_right_shoulder" : 8,
        #             "front_right_leg": 9,
        #             "front_right_foot": 11,
        #             "rear_left_shoulder": 13,
        #             "rear_left_leg": 14,
        #             "rear_left_foot": 16,
        #             "rear_right_shoulder": 18,
        #             "rear_right_leg": 19,
        #             "rear_right_foot": 21}

        self.joint_dic = {"front_left_shoulder":2,
                    "front_left_leg":3,
                    "front_left_foot": 5,
                    "front_right_shoulder" : 7,
                    "front_right_leg": 8,
                    "front_right_foot": 10,
                    "rear_left_shoulder": 12,
                    "rear_left_leg": 13,
                    "rear_left_foot": 15,
                    "rear_right_shoulder": 17,
                    "rear_right_leg": 18,
                    "rear_right_foot": 20}
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
        self.angle_unit = angle_unit

        self.initial_ef_positions = initial_ef_positions
        

        # Pybullet Setup
        # Prepare environment
        self.prep_environment(plane_orientation=[0, 0, 0], center_plane=center_plane)

        # Load the quadruped
        self.robotId, self.num_joints = self.load_quadruped(self.urdf_path, center, p.getQuaternionFromEuler(orientation))

        # Display Initial Pose 
        self.move_robot_to_pose(self.robotId, initial_theta, self.angle_unit)

        # Kinematics Controller
        self.kin_solver = kinematics_old.Kinematics(self.length, 
                                                self.width, 
                                                self.l1, 
                                                self.l2, 
                                                self.l3, 
                                                self.l4)

        # Gait Controller
        self.gait_controller = gait.GaitController(initial_ef_positions=self.initial_ef_positions,
                                                   initial_theta=self.initial_theta,
                                                   initial_center=self.center_kin,
                                                   initial_orientation=self.orientation_kin)

        self._leg_joint_map = {
            "FL": [self.joint_dic["front_left_shoulder"],  self.joint_dic["front_left_leg"],  self.joint_dic["front_left_foot"]],
            "FR": [self.joint_dic["front_right_shoulder"], self.joint_dic["front_right_leg"], self.joint_dic["front_right_foot"]],
            "RL": [self.joint_dic["rear_left_shoulder"],   self.joint_dic["rear_left_leg"],   self.joint_dic["rear_left_foot"]],
            "RR": [self.joint_dic["rear_right_shoulder"],  self.joint_dic["rear_right_leg"],  self.joint_dic["rear_right_foot"]],
        }
        self._leg_order = ["FL", "FR", "RL", "RR"]

        # Start simulation
        self.initial_state = p.saveState()
        self.start_simulation()
        
    def load_quadruped(self, urdf_path, center, orn):
        """
        Load the urdf with its meshes
        
        :param urdf_path: file path
        :param center: in pybullet frame
        :param orn: in pybullet frame
        """
        try:
            robotId = p.loadURDF(urdf_path, center, orn, useFixedBase=False)
            print(f"Successfully loaded {urdf_path}!")
            num_joints = p.getNumJoints(robotId)
            print(f"Robot has {num_joints} joints.")
            for i in range(num_joints):
                info = p.getJointInfo(robotId, i)
                joint_name = info[1].decode("utf-8")          
                print(f"Loaded Joint: {joint_name} (ID: {i})")
            return robotId, num_joints
        except Exception as e:
            print(f"Error loading URDF: {e}")
            return
        
    def prep_environment(self, plane_orientation, center_plane, cameraDistance=1, cameraYaw=-181, cameraPitch=-165, cameraTargetPosition=[0, 0, 0]):
        """
        Docstring for prep_environment
        
        :param plane_orientation: in pybullet frame
        :param center_plane: in pybullet frame
        :param cameraDistance: 
        :param cameraYaw: 
        :param cameraPitch: 
        :param cameraTargetPosition: 
        """
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.resetDebugVisualizerCamera(cameraDistance=cameraDistance, cameraYaw=cameraYaw, cameraPitch=cameraPitch, cameraTargetPosition=cameraTargetPosition)
        self.roll_slider = p.addUserDebugParameter("----roll", -60, 60, 0)
        self.pitch_slider = p.addUserDebugParameter("----pitch", -60, 60, 0)
        self.yaw_slider = p.addUserDebugParameter("----yaw", -60, 60, 0)



        orn = p.getQuaternionFromEuler(plane_orientation)  # Roll, Pitch, Yaw in radians
        p.loadURDF("plane.urdf", center_plane, orn, useFixedBase=True)

    def get_imu_data(self):
        """
        Returns (roll, pitch) in radians and (roll_rate, pitch_rate)
        simulating an onboard IMU and Gyroscope.
        """
        # Orientation (Quaternion -> Euler)
        pos, orn = p.getBasePositionAndOrientation(self.robotId)
        roll, pitch, yaw = p.getEulerFromQuaternion(orn)
        
        # Angular Velocity (Gyroscope)
        # lin_vel, ang_vel = p.getBaseVelocity(self.robotId)
        # # ang_vel is [wx, wy, wz] (roll_rate, pitch_rate, yaw_rate)
        # roll_rate = ang_vel[0]
        # pitch_rate = ang_vel[1]
        
        return roll, pitch, yaw

    def get_ef_positions(self):
        """
        Returns the end effector positions in pybullet frame 
        """
        ef_indices = [7, 12, 17, 22]
        ef_positions = []
        for i in ef_indices:
            ef_positions.append(p.getLinkState(self.robotId, i)[0])
        return ef_positions

    def move_robot_to_pose(self, robotId, theta, unit='degrees'):
        """
        Move the robot to a given pose.
        
        :param robotId: 
        :param theta: angles for all legs [[FL], [FR], [RL], [RR]]
        :param unit: angle unit
        """

        if unit == 'degrees':
            theta = [math.radians(angle) for angle in theta]
        # Convert angles to radians and apply directions
        theta = [angle * dir for angle, dir in zip(theta, self.theta_dirs)]
        p.setJointMotorControlArray(robotId,
                                    jointIndices=list(self.joint_dic.values()),
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=theta)
        p.stepSimulation()
        time.sleep(1./240.)
    
    # NOT USED
    def execute_leg_trajectory(self, trajectory, leg="FL"):
        """ 
        Execute a trajectory for a single leg. Only for Testing.

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
            for _ in range(3): 
                p.stepSimulation()
                time.sleep(1./240.)
    
    # NOT USED
    def execute_robot_trajectory(self, trajectories):
        """
        For testing
        
        :param trajectories: FL, FR, RL, RR
        """
        fl_trajectory = trajectories[0]
        fr_trajectory = trajectories[1]
        rl_trajectory = trajectories[2]
        rr_trajectory = trajectories[3]

        # # Apply directions
        # fl_trajectory = [angle * dir for angle, dir in zip(fl_trajectory, self.theta_dirs[:3])]
        # fr_trajectory = [angle * dir for angle, dir in zip(fr_trajectory, self.theta_dirs[3:6])]
        # rl_trajectory = [angle * dir for angle, dir in zip(rl_trajectory, self.theta_dirs[6:9])]
        # rr_trajectory = [angle * dir for angle, dir in zip(rr_trajectory, self.theta_dirs[9:12])]


        for fl_angle, fr_angle, rl_angle, rr_angle in zip(fl_trajectory, fr_trajectory, rl_trajectory, rr_trajectory):
            # Apply theta dirs
            fl_angle = [angle * dir for angle, dir in zip(fl_angle, self.theta_dirs[:3])]
            p.setJointMotorControlArray(self.robotId,
                            jointIndices=[3, 4, 6],  # shoulder, leg, foot
                            controlMode=p.POSITION_CONTROL,
                            targetPositions=fl_angle)
            
            fr_angle = [angle * dir for angle, dir in zip(fr_angle, self.theta_dirs[3:6])]
            p.setJointMotorControlArray(self.robotId,
                            jointIndices=[8, 9, 11],  # shoulder, leg, foot
                            controlMode=p.POSITION_CONTROL,
                            targetPositions=fr_angle)
            
            rl_angle = [angle * dir for angle, dir in zip(rl_angle, self.theta_dirs[6:9])]
            p.setJointMotorControlArray(self.robotId,
                            jointIndices=[13, 14, 16],  # shoulder, leg, foot
                            controlMode=p.POSITION_CONTROL,
                            targetPositions=rl_angle)
            
            rr_angle = [angle * dir for angle, dir in zip(rr_angle, self.theta_dirs[9:12])]
            p.setJointMotorControlArray(self.robotId,
                            jointIndices=[18, 19, 21],  # shoulder, leg, foot
                            controlMode=p.POSITION_CONTROL,
                            targetPositions=rr_angle)
            
            p.stepSimulation()
            time.sleep(1./240.)
        
    def move_callback(self, leg, angles, unit="rad"):
        """Apply IK angles for one leg to pybullet joints.

        :param leg:    leg name FL / FR / RL / RR
        :param angles: [shoulder, leg, foot] in radians (or degrees)
        :param unit:   "rad" or "degrees"
        """
        if unit == "degrees":
            angles = [math.radians(a) for a in angles]
        leg_idx = self._leg_order.index(leg)
        dirs = self.theta_dirs[leg_idx * 3: (leg_idx + 1) * 3]
        angles = [a * d for a, d in zip(angles, dirs)]
        p.setJointMotorControlArray(self.robotId, self._leg_joint_map[leg], p.POSITION_CONTROL, angles)

    def start_simulation(self):
        
        
        # Start Simulation Loop
        while True:

            # Step Simulation and add small delay for smooth transition to initial pose
            p.stepSimulation()
            time.sleep(1./240.)
            
            
            # Keyboard and Mouse Events
            keyboard_event = p.getKeyboardEvents()
            # print(f"{keyboard_event})

            self.upArrowKey = 65297
            self.downArrowKey = 65298
            self.leftArrowKey = 65295
            self.rightArrowKey = 65296
            self.cKey = ord('c')
            self.rKey = ord('r')
            self.eKey = ord('e')
            self.wKey = ord('w')
            self.sKey = ord('s')
            self.aKey = ord('a')
            self.dKey = ord('d')
            self.qKey = ord('q')
            self.iKey = ord('i')
            self.tKey = ord('t')

            # Not used at the moment
            # mouse_event = p.getMouseEvents()
            # try:
            #     event_type = mouse_event[0][0]
            #     button_state = mouse_event[0][-1]
            #     button_index = mouse_event[0][3]
            # except:
            #     # keyboard_event = None
            #     event_type = None
            #     button_state = None

            # Print imu data
            if self.key_is_pressed(keyboard_event, self.iKey):
                imu_data_raw = self.get_imu_data()
                imu_data_kin = from_pybullet_orn(imu_data_raw)
                
                roll = imu_data_kin[0]
                pitch = imu_data_kin[1]
                yaw = imu_data_kin[2]

                # roll_deg  = np.degrees(roll)
                # pitch_deg = np.degrees(pitch)
                # yaw_deg   = np.degrees(yaw)
                print("---------------")
                # print(f"roll: {roll_deg}\npitch: {pitch_deg}\nyaw: {yaw_deg}")
                print(f"roll: {roll}\npitch: {pitch}\nyaw: {yaw}")

            # Test certain trajectories 
            if self.key_is_pressed(keyboard_event, self.tKey):
                pass

            # Move to a certain pose
            if self.key_is_pressed(keyboard_event, self.cKey):
                roll_angle = np.radians(p.readUserDebugParameter(self.roll_slider))
                pitch_angle = np.radians(p.readUserDebugParameter(self.pitch_slider))
                yaw_angle = np.radians(p.readUserDebugParameter(self.yaw_slider))
                angles = self.kin_solver.robot_IK(self.center_kin, [roll_angle, pitch_angle, yaw_angle], self.initial_ef_positions)
                self.move_robot_to_pose(self.robotId, angles, unit="rad")
                
            # Reset scene
            if self.key_is_pressed(keyboard_event, self.eKey):
                self.respawn_robot()
            
            # Reset to initial pose
            if self.key_is_pressed(keyboard_event, self.rKey):
                print("RESETTING TO INITIAL POSE")
                angles = self.kin_solver.robot_IK(self.center_kin, [0, 0, 0], self.initial_ef_positions)
                self.move_robot_to_pose(self.robotId, angles, unit="rad")

            # Go Forward
            if self.key_is_pressed(keyboard_event, self.wKey) or self.key_is_pressed(keyboard_event, self.upArrowKey):
                self.move(0.26, "+x")
                               
            # Go Backward
            if self.key_is_pressed(keyboard_event, self.sKey) or self.key_is_pressed(keyboard_event, self.downArrowKey):
                self.move(0.26, "-x")
                        
            # Go Left
            if self.key_is_pressed(keyboard_event, self.aKey) or self.key_is_pressed(keyboard_event, self.leftArrowKey):
                self.move(0.26, "-y")
  
            # Go Right
            if self.key_is_pressed(keyboard_event, self.dKey) or self.key_is_pressed(keyboard_event, self.rightArrowKey):
                self.move(0.26, "+y")

    def respawn_robot(self):
        # I am inevitable
        # p.resetSimulation() # breaks everything

        # Position the robot at its initial position
        p.restoreState(stateId = self.initial_state)
        # Reset the camera
        p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=-181, cameraPitch=-165, cameraTargetPosition=[0, 0, 0])
        print("RESETTING SCENE")
        # Make sure the robot has the initial pose
        angles = self.kin_solver.robot_IK(self.center_kin, [0, 0, 0], self.initial_ef_positions)
        self.move_robot_to_pose(self.robotId, angles, unit="rad")
        
    def move(self, velocity, direction):
        
        if direction == "+x":
            print("GOING FORWARDS")
        elif direction == "-x":
            print("GOING BACKWARDS")
        elif direction == "+y":
            print("GOING RIGHT")
        elif direction == "-y":
            print("GOING LEFT")
        deceleration_flag = False

        current_time = 0

        while True:
            current_time += 1./240.
            p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=-181, cameraPitch=-165, cameraTargetPosition=p.getBasePositionAndOrientation(self.robotId)[0])
            keyboard_event = p.getKeyboardEvents()

            if self.key_is_pressed(keyboard_event, self.qKey):
                deceleration_flag = True
                print("DECELERATING")
            T_cycle = 0.28
            duty_factor = 0.5
            swing_height = 0.035
            ef_vel = self.gait_controller.trot(
                            current_time, TIME_STEP, T_cycle, duty_factor, velocity, swing_height,
                            self.get_imu_data(), dir=direction, deceleration_flag=deceleration_flag,
                            move_callback=self.move_callback)
            p.stepSimulation()
            time.sleep(TIME_STEP)
            
            # Once speed is 0 return to default pose
            if ef_vel == 0.0 and deceleration_flag:
                print("STOPPED")
                angles = self.kin_solver.robot_IK(self.center_kin, [0, 0, 0], self.initial_ef_positions)
                self.move_robot_to_pose(self.robotId, angles, unit="rad")
                break

    def key_is_pressed(self, keyboard_event, key):
        return key in keyboard_event and keyboard_event[key]&p.KEY_WAS_TRIGGERED
          
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

    # Degrees
    theta = [0, -30, 60, # FL
             0, -30, 60, # FR
             0, -30, 60, # RL
             0, -30, 60 ] # RR
    
    # X forward Y up Z left in milimeters
    ef_positions = np.array([
            [ 95, 48.13,  105, 1], # FL
            [ 95, 48.13,  -105, 1], # FR
            [-45, 48.13, 105, 1], # RL
            [-45, 48.13, -105, 1] # RR
            ])
    
    # X forward Y up Z left in milimeters
    ef_positions2 = np.array([
        [67.29, 46.12, 107, 1],
        [67.29, 46.12, -107, 1],
        [-72.21, 46.12, 107, 1],
        [-72.21, 46.12, -107, 1]
        ])


    test = kinematics_old.Kinematics()
    print(test.robot_FK([0, 250, 0], [0, 0, 0],theta, unit="degrees"))

    pybullet_sim = PybulletSim(length=kinematics_old.LENGTH, 
                               width=kinematics_old.WIDTH,
                               l1=kinematics_old.L1, 
                               l2=kinematics_old.L2, 
                               l3=kinematics_old.L3, 
                               l4=kinematics_old.L4,
                               center=center,
                               orientation=orientation,
                               center_plane=center_plane,
                               initial_theta=theta,
                               initial_ef_positions=ef_positions2,
                               angle_unit="degrees")
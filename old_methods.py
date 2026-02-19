    def go_forward(self, current_time, velocity=0.9, deceleration_flag=False):

        T_cycle = 0.2
        duty_factor = 0.5
        swing_height = 0.03
        velocity = velocity
        
        return self.gait_controller.trot(current_time, 
                                  T_cycle, 
                                  duty_factor, 
                                  velocity, 
                                  swing_height, 
                                  p, 
                                  self.robotId, 
                                  self.get_imu_data(), 
                                  dir="+x", 
                                  deceleration_flag=deceleration_flag)

    def go_backward(self, current_time, velocity=0.9, deceleration_flag=False):
        T_cycle = 0.2
        duty_factor = 0.5
        swing_height = 0.03
        velocity = velocity
        
        return self.gait_controller.trot(current_time, 
                                  T_cycle, 
                                  duty_factor, 
                                  velocity, 
                                  swing_height, 
                                  p, 
                                  self.robotId, 
                                  self.get_imu_data(), 
                                  dir="-x",
                                  deceleration_flag=deceleration_flag)

    def go_left(self, current_time, velocity=0.9, deceleration_flag=False):
        T_cycle = 0.2
        duty_factor = 0.5
        swing_height = 0.03
        velocity = velocity
        
        return self.gait_controller.trot(current_time, 
                                  T_cycle, 
                                  duty_factor, 
                                  velocity, 
                                  swing_height, 
                                  p, 
                                  self.robotId, 
                                  self.get_imu_data(), 
                                  dir="+y", 
                                  deceleration_flag=deceleration_flag)

    def go_right(self, current_time, velocity=0.9, deceleration_flag=False):
        T_cycle = 0.2
        duty_factor = 0.5
        swing_height = 0.03
        velocity = velocity
        
        return self.gait_controller.trot(current_time, 
                                  T_cycle, 
                                  duty_factor, 
                                  velocity, 
                                  swing_height, 
                                  p, 
                                  self.robotId, 
                                  self.get_imu_data(), 
                                  dir="-y",
                                  deceleration_flag=deceleration_flag)   




def legIK(self, point, side="r"):
        """
        DERIVED IK: Solves for the exact DH parameters used in legFK.
        Input: (x, y, z) in ROS frame relative to shoulder.
        """
        # 1. Transform ROS point (X:Fwd, Y:Left, Z:Up) back to raw DH frame
        # This is the inverse of the T_BaseFrame used in FK
        xdh, ydh, zdh = -point[2], point[1], point[0]
        d1 = self.l1 if side == "r" else -self.l1

        # 2. Solve for Theta 1 (Hip)
        # Equation: ydh = (L2c2 + L3c23)s1 - d1c1 | xdh = (L2c2 + L3c23)c1 + d1s1
        dist_sq = xdh**2 + ydh**2
        if dist_sq < d1**2: return [0,0,0] # Target inside the hip "dead zone"
        
        # This is the algebraic solution for theta1 in the DH chain
        theta1 = math.atan2(ydh, xdh) + math.asin(d1 / math.sqrt(dist_sq))

        # 3. Solve for Theta 3 (Knee)
        # Find the projection B (distance from hip axis to foot in the leg plane)
        B = xdh * math.cos(theta1) + ydh * math.sin(theta1)
        # Using Law of Cosines for a 2R arm (L2, L3) reaching (B, zdh)
        D = (B**2 + zdh**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        D = np.clip(D, -1.0, 1.0)
        theta3 = math.acos(D)

        # 4. Solve for Theta 2 (Thigh)
        theta2 = math.atan2(zdh, B) - math.atan2(self.l3 * math.sin(theta3), self.l2 + self.l3 * math.cos(theta3))

        return [theta1, theta2, theta3]


def legIK(self, point, side="r"):
        """ Inverse Kinematics for a single leg 
        
        :param point: Target foot position (x, y, z) relative to shoulder
        
        :returns: Tuple of joint angles (theta1, theta2, theta3)"""
        (x, y, z) = (-point[2], point[1], point[0])
        
        D = (y**2 + (-z)**2 - self.l1**2 + (-x)**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)


        if D > 1 or D < -1:
            # DOMAIN BREACHED
            print("---------DOMAIN BREACH---------")
            D = np.clip(D, -1.0, 1.0)

        if side=="l":
            theta3 = np.atan2(-np.sqrt(1 - D**2), D)
        else:
            theta3 = np.atan2(np.sqrt(1 - D**2), D)
        
        sqrt_component = y**2 + (-z)**2 - self.l1**2
        if sqrt_component < 0.0:
            print("NEGATIVE SQRT")
            sqrt_component = 0.0

        theta1 = -np.arctan2(z, y) - np.arctan2(
            np.sqrt(sqrt_component), -self.l1)
        
        theta2 = np.arctan2(-x, np.sqrt(sqrt_component)) - np.arctan2(
            self.l3 * np.sin(theta3),
            self.l2 + self.l3 * np.cos(theta3))
        
        joint_angles = np.array([theta1, theta2, theta3])
        return joint_angles
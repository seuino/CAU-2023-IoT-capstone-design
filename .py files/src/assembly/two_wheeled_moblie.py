pi = 3.141592


class TwoWheeledMobile:
    def __init__(self, sonar, *dcmotors):
        self._INTERMOTOR_DIST = 115. # mm
        self._WHEEL_DIA = 43. # mm

        self.linear_x = 0.25 # m/s
        # self.linear_y
        # self.linear_z
        # self.angular_x
        # self.angular_y
        self.angular_z = 0 # rad/s (counter-clockwise)

        self._left_target_linear_vel = 0 # m/s
        self._right_target_linear_vel = 0
        self._left_target_angular_vel = 0 # rad/s
        self._right_target_angular_vel = 0

        self.left_dcmotor = dcmotors[0]
        self.right_dcmotor = dcmotors[1]

        self.sonar = sonar

    def get_cmd_vel(self, data):
        self.linear_x = data.linear.x # m/s
        # self.linear_y = data.linear.y
        # self.linear_z = data.linear.z
        # self.angular_x = data.angular.x
        # self.angular_y = data.angular.y
        self.angular_z = data.angular.z # rad/s

    def achieve_cmd_vel(self):
        self._left_target_linear_vel = self.linear_x \
            - (self.angular_z * self._INTERMOTOR_DIST*1e-3/2)
        self._right_target_linear_vel = self.linear_x \
            + (self.angular_z * self._INTERMOTOR_DIST*1e-3/2)
        
        self._left_target_angular_vel = self._left_target_linear_vel / (self._WHEEL_DIA*1e-3*pi)
        self._right_target_angular_vel = self._right_target_linear_vel / (self._WHEEL_DIA*1e-3*pi)

        self.left_dcmotor.control_pid(self._left_target_angular_vel)
        self.right_dcmotor.control_pid(self._right_target_angular_vel)
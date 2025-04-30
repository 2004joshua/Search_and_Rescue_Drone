# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Example of Python controller for Mavic patrolling around the house.
   Open the robot window to see the camera view.
   This demonstrates how to go to specific world coordinates using its GPS, imu and gyroscope.
   The drone reaches a given altitude and patrols from waypoint to waypoint."""

from controller import Robot
import sys
import cv2

try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class Mavic (Robot):
    # Constants, empirically found.
    K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts.
    # Vertical offset where the robot actually targets to stabilize itself.
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0        # P constant of the vertical PID.
    K_ROLL_P = 50.0           # P constant of the roll PID.
    K_PITCH_P = 30.0          # P constant of the pitch PID.

    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1
    # Precision between the target position and the robot position in meters
    target_precision = 0.5

    X_MAX = -60 
    X_MIN = -30 
    Y_MIN = 5
    Y_MAX = 20 

    LINESPACING = 2.0

    def __init__(self):
        Robot.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        # Camera
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        self.camera_width = self.camera.getWidth()
        self.camera_height = self.camera.getHeight()

        # GPS
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)

        # Motors
        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        
        # Camera pitch motor: Controls camera angle
        self.camera_pitch_motor = self.getDevice("camera pitch")
        self.camera_pitch_motor.setPosition(0.4)
        
        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        self.current_pose = 6 * [0]  # X, Y, Z, yaw, pitch, roll
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.target_altitude = 0
        
        self.avoidance_mode = False
        self.clear_counter = 0
        self.CLEAR_THRESHOLD = 10
        self.AVOID_THRESHOLD = 1
        self.obs_counter = 0
        self.AVOID_BLEND = 0.3

    def set_position(self, pos):
        """
        Set the new absolute position of the robot
        Parameters:
            pos (list): [X,Y,Z,yaw,pitch,roll] current absolute position and angles
        """
        self.current_pose = pos

    def get_frame(self):
        """
        Grab the current camera image from Webots and return
        it as a HxWx3 numpy array.
        """
        raw_image = self.camera.getImage()
        arr = np.frombuffer(raw_image, np.uint8)
        arr = arr.reshape((self.camera_height, self.camera_width, 4))
        return arr[:, :, :3]  # Discard the alpha channel
    
    def detect_obstacle(self, img):
        """
        Given a BGR image, detect if theres an obstacle
        Returns: 
            (has_obstacle: bool, avoid_yaw: float, avoid_pitch: float)
        """
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0,100,100), (10,255,255))

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not cnts:
            return False, 0.0, 0.0
        
        c = max(cnts, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        center_x = x + w // 2

        if center_x < self.camera_width * 0.4:
            yaw_offset, pitch_offset = +0.3, 0.0
        elif center_x > self.camera_width * 0.6:
            yaw_offset, pitch_offset = -0.3, 0.0
        else:
            yaw_offset, pitch_offset = 0.0, 0.3
        
        return True, yaw_offset, pitch_offset

    def move_to_target(self, waypoints, verbose_movement=False, verbose_target=False):
        """
        Move the robot to the given coordinates
        Parameters:
            waypoints (list): list of X,Y coordinates
            verbose_movement (bool): whether to print remaning angle and distance or not
            verbose_target (bool): whether to print targets or not
        Returns:
            yaw_disturbance (float): yaw disturbance (negative value to go on the right)
            pitch_disturbance (float): pitch disturbance (negative value to go forward)
        """

        if self.target_position[0:2] == [0, 0]:  # Initialization
            self.target_position[0:2] = waypoints[0]
            if verbose_target:
                print("First target: ", self.target_position[0:2])

        # if the robot is at the position with a precision of target_precision
        if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(self.target_position, self.current_pose[0:2])]):

            self.target_index += 1
            if self.target_index > len(waypoints) - 1:
                self.target_index = 0
            self.target_position[0:2] = waypoints[self.target_index]
            if verbose_target:
                print("Target reached! New target: ",
                      self.target_position[0:2])

        # This will be in ]-pi;pi]
        self.target_position[2] = np.arctan2(
            self.target_position[1] - self.current_pose[1], self.target_position[0] - self.current_pose[0])
        # This is now in ]-2pi;2pi[
        angle_left = self.target_position[2] - self.current_pose[5]
        # Normalize turn angle to ]-pi;pi]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if (angle_left > np.pi):
            angle_left -= 2 * np.pi

        # Turn the robot to the left or to the right according the value and the sign of angle_left
        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)
        # non proportional and decreasing function
        pitch_disturbance = clamp(
            np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1)

        if verbose_movement:
            distance_left = np.sqrt(((self.target_position[0] - self.current_pose[0]) ** 2) + (
                (self.target_position[1] - self.current_pose[1]) ** 2))
            print("remaning angle: {:.4f}, remaning distance: {:.4f}".format(
                angle_left, distance_left))
        return yaw_disturbance, pitch_disturbance

    def generate_lawnmower(self, min_x, max_x, min_y, max_y, spacing):
        waypoints = []
        y = min_y
        left_to_right = True
        while y <= max_y:
            if left_to_right:
                waypoints.append([min_x, y])
                waypoints.append([max_x, y])
            else:
                waypoints.append([max_x, y])
                waypoints.append([min_x, y])
            
            y += spacing
            left_to_right = not left_to_right
        for i in range(len(waypoints)):
            print("waypoint {}: {}".format(i, waypoints[i]))
        return waypoints
    
    def run(self):
        t1 = self.getTime()

        waypoints = self.generate_lawnmower(
            self.X_MIN, self.X_MAX,
            self.Y_MIN, self.Y_MAX,
            self.LINESPACING)

        # target altitude of the robot in meters
        self.target_altitude = 5.0
        
        self.idx = 0
        self.prev_has_obs = False
	
        while self.step(self.time_step) != -1:

            # 1) Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            if self.idx % 100 == 0:
                print(f"GPS: {x_pos:.2f}, {y_pos:.2f}, {altitude:.2f} | IMU: {roll:.2f}, {pitch:.2f}, {yaw:.2f}")
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])
                 
            # 3) Vertical PID (always on)
            vertical_error = self.target_altitude - altitude + self.K_VERTICAL_OFFSET
            vertical_input = self.K_VERTICAL_P * pow(clamp(vertical_error, -1, 1), 3.0)

            # reset steering inputs
            yaw_input = 0.0
            pitch_nav_input = 0.0
            yaw_disturbance = 0.0
            pitch_disturbance = 0.0

            # 4) Only nav once we're at height
            if altitude > self.target_altitude - 1.0:
                if self.getTime() - t1 > 0.1:
                    NAV_GAIN = 5.0
                    yaw_disturbance, pitch_disturbance = self.move_to_target(waypoints)
                    yaw_disturbance = NAV_GAIN * yaw_disturbance
                    pitch_disturbance = NAV_GAIN * pitch_disturbance
                    t1 = self.getTime()
                    
				# 2) Always get camera frame & detect
                frame = self.get_frame()
                has_obs, avoid_yaw, avoid_pitch = self.detect_obstacle(frame)
                if self.idx % 100 == 0 and has_obs != self.prev_has_obs:
                    print(f"detect_obstacle → has_obs={has_obs}, yaw_off={avoid_yaw:.2f}, pitch_off={avoid_pitch:.2f}")
                    self.prev_has_obs = has_obs
				
                if has_obs:
                    self.obs_counter += 1
                    self.clear_counter = 0
                else:
                    self.clear_counter += 1
                    self.obs_counter = 0
            
                if not self.avoidance_mode and self.obs_counter >= self.AVOID_THRESHOLD:
                    print("Obstacle detected, entering avoidance mode")
                    self.avoidance_mode = True
                elif self.avoidance_mode and self.clear_counter >= self.CLEAR_THRESHOLD:
                    print("Obstacle cleared, exiting avoidance mode")
                    self.avoidance_mode = False
                
				# blend nav + avoidance
                if self.avoidance_mode:
                    yaw_input   = yaw_disturbance + avoid_yaw * self.AVOID_BLEND
                    pitch_nav_input = pitch_disturbance + avoid_pitch * self.AVOID_BLEND
                else:
                    yaw_input   = yaw_disturbance
                    pitch_nav_input = pitch_disturbance

            # 5) Roll/Pitch stabilization
            roll_input  = self.K_ROLL_P  * clamp(roll,  -1, 1) + roll_acceleration
            pitch_stab  = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration

            # 6) Mix into motors
            fl = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_nav_input - roll_input
            fr = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_nav_input + roll_input
            rl = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_stab - roll_input
            rr = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_stab + roll_input

            # 7) Send commands
            self.front_left_motor .setVelocity(fl)
            self.front_right_motor.setVelocity(-fr)
            self.rear_left_motor  .setVelocity(-rl)
            self.rear_right_motor .setVelocity(rr)
            self.idx += 1


# To use this controller, the basicTimeStep should be set to 8 and the defaultDamping
# with a linear and angular damping both of 0.5
robot = Mavic()
robot.run()

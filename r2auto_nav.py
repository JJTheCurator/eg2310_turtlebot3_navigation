# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import math
import scipy.stats

# constants
rotatechange = 0.1
speedchange = 0.05
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.25
#front_angle = 179
front_angle = 0
front_angles = range(front_angle - 10,front_angle + 10,1)

right_angle = 269
left_angle = 79
back_angle = 179
back_angles = range(back_angle - 10,back_angle + 10,1)

ROTATE_BIAS_DEGREE = 0
DRINK_PRESENT = True
DRINK_NOT_PRESENT = False
DEBUG = False

occ_bins = [-1, 0, 50, 100]
map_bg_color = 1

scanfile = 'lidar.txt'
mapfile = 'map.txt'

# forward_op = [
#     [(0.30, 0)],
#     [(0.30, -90), (1.120, 0)],
#     [(1.35, -90), (1.120, 0)],
#     [(1.35, -90), (0.40, 0)],
#     [(1.35, -90), (0.40, -90), (0.40, 90), (0.40, 90), (0.30, 0)],
#     [(0.30, -90), (0.40, 90)],
# ]

# forward_op = [
#     [(1.55, 0.30, 0)],
#     [(1.55, 0.30, -90), (0.8, 1.120, 0)],
#     [(0.5, 1.35, -90), (0.8, 1.120, 0)],
#     [(0.5, 1.35, -90), (1.60, 0.40, 0)],
#     [(0.15, 1.55, -90), (2.440, 0.5, 90), (1.58, 0.4, 0)],
#     [(1.55, 0.30, -90), (1.60, 0.40, 90), (0.92, 1.2, 0)],
# ]

half_pi = math.pi / 2

# (travelling distance, lidar checking distance, turning angle)
forward_op = [
    [(1.53, 0.27, 0)],
    [(1.53, 0.27, -half_pi), (0.9, 1.10, 0)],
    [(1.25, 0.55, -half_pi), (0.8, 1.120, 0)],
    [(1.25, 0.55, -half_pi), (1.60, 0.55, 0)],
    [(0.16, 1.5, -half_pi), (2.380, 0.45, half_pi), (1.55, 0.25, 0)],
    [(1.55, 0.30, -half_pi), (1.60, 0.40, half_pi), (0.80, 1.4, 0)],
]

reverse_op = [
    [(1.55, 0.05, 0)],
    [(0.9, 0.5, half_pi), (1.6, 0.15, 0)],
    [(0.8, 0.5, half_pi), (1.25, 0.15, 0)],
    [(1.6, 0.5, half_pi), (1.25, 0.15, 0)],
    [(1.5, 0.6, -half_pi), (2.340, 0.5, half_pi), (0.15, 0.10, 0)],
    [(1.6, 0.4, -half_pi), (1.8, 0.4, -half_pi), (1.65, 0.5, half_pi), (1.6, 0.2, 0)],
]

# reverse_op = [
#     [()],
#     [(0.8, 0.5, half_pi)],
#     [(0.8, 0.5, half_pi)],
#     [(1.6, 0.5, half_pi)],
#     [(1.58, 0.4, -half_pi), (2.440, 0.5, half_pi)],
#     [(1.6, 0.4, half_pi), (1.4, 0.4, half_pi), (0.8, 0.5, half_pi)],
# ]



# kp = 0.1
# ki = 0.05
# kd = 0.05

# kp = 0.1
# ki = 0.09
# kd = 0.12

kp = 0.1
ki = 0.1
kd = 0.38

#ki = 0.08
# kd = 0.362

#ki = 0.11
#kd = 0.35

#wall following pid
kp_w = 300.0
kd_w = 0.1
ki_w = 0.11

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',5)
        #self.check_drink_publisher = self.create_publisher(String, 'check_drink',10)
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.imu_subscription = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.imu_subscription

        # create subscription to track occupancy
        # self.occ_subscription = self.create_subscription(
        #     OccupancyGrid,
        #     'map',
        #     self.occ_callback,
        #     qos_profile_sensor_data)
        # self.occ_subscription  # prevent unused variable warning
        # self.occdata = np.array([])
        # self.tfBuffer = tf2_ros.Buffer()
        # self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        self.one_meter_counter = 0

        # self.push_button_subscription = self.create_subscription(
        #     String,
        #     'push_button',
        #     self.push_button_callback,
        #     10,
        # )
        # self.push_button_subscription

        self.weight_sensor_subscription = self.create_subscription(
            Float32,
            'weight_sensor',
            self.weight_sensor_callback,
            10,
        )
        self.weight_sensor_subscription

        # self.nfc_subscription = self.create_subscription(
        #     String,
        #     "nfc",
        #     self.nfc_callback,
        #     10,
        # )
        # self.nfc_subscription

        #UI logic
        self.table_number = 0
        self.table_input = "0"

        #navigation logic
        self.speed = 0.0
        self.first_run = True
        self.total_distance = 0.0
        self.x = 0.0
        self.y = 0.0
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.linear_distance = 0.0

        self.first_yaw = 0.0
        self.second_yaw = 0.0
        #drink can logic
        self.is_drink_present = False
        self.tick = 0
        self.lidar_tick = 0
        self.current_tick = 0

        self.grid_angle = 0.0
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        #time.sleep(0.5)
        rclpy.spin_once(self)
        self.grid_angle = self.yaw
        if(self.laser_range.size != 0):
            self.wall_distance = self.laser_range[left_angle]
        else:
            self.wall_distance = 0.55
        #print(f"self.grid_angle: {self.grid_angle}\nself.wall_distance: {self.wall_distance}")
        #input("init check[enter]")

        #pid for 
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0
        self.prev_error = 0.0
        self.error_sum = 0.0

        #pid for wall following
        self.w_p_error = 0.0
        self.w_i_error = 0.0
        self.w_d_error = 0.0
        self.w_prev_error = 0.0
        self.w_error_sum = 0.0


    def check_drink(self, exit_condition):
        #print("in check_drink")
        while rclpy.ok():
            #self.check_drink_publisher.publish(String())
            rclpy.spin_once(self)
            if(self.is_drink_present == exit_condition):
                print("Drink can is detected. Proceeding.")
                break
            time.sleep(0.005)
        # if(exit_condition == DRINK_NOT_PRESENT):
            # print("Drink is not present, proceeding.")
        # else:
            # print("Drink is present, proceeding")

    def imu_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        
        #print(f"odom self.yaw: {self.yaw}\nself.grid_angle: {self.grid_angle}")
        self.tick += 1
        #print(f"self.previous_x = {self.previous_x}")
        #print(f"self.previous_y = {self.previous_y}")
        #print(f"self.linear_distance = {self.linear_distance}")

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        #orientation_quat =  msg.pose.pose.orientation
        #self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        if(self.first_run):
            self.linear_distance = 0
            self.previous_x = msg.pose.pose.position.x
            self.previous_y = msg.pose.pose.position.y
            self.first_run = False
            self.error_sum = 0
            self.prev_error = 0
            self.w_prev_error = 0
            self.w_error_sum = 0
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        d_increment = math.sqrt((x - self.previous_x) ** 2 +
                   (y - self.previous_y) ** 2)

        self.linear_distance += d_increment
        self.previous_x = msg.pose.pose.position.x
        self.previous_y = msg.pose.pose.position.y

        #print(f"odom self.yaw: {self.yaw}\nself.grid_angle: {self.grid_angle}")
        #self.tick += 1
        #print(f"self.previous_x = {self.previous_x}")
        #print(f"self.previous_y = {self.previous_y}")
        #print(f"self.linear_distance = {self.linear_distance}")

    def occ_callback(self, msg):
        # create numpy array
        occdata = np.array(msg.data)
        # compute histogram to identify bins with -1, values between 0 and below 50, 
        # and values between 50 and 100. The binned_statistic function will also
        # return the bin numbers so we can use that easily to create the image 
        oc2 = occdata + 1
        self.occdata = np.uint8(oc2.reshape(msg.info.height, msg.info.width))
        #np.savetxt(mapfile, self.occdata)
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
        # get width and height of map
        iwidth = msg.info.width
        iheight = msg.info.height
        # calculate total number of bins
        total_bins = iwidth * iheight
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0], occ_counts[1], occ_counts[2], total_bins))

        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return
            
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        print('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
        # convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        print('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))

        # get map resolution
        # map_res = msg.info.resolution
        # # get map origin struct has fields of x, y, and z
        # map_origin = msg.info.origin.position
        # # get map grid positions for x, y position
        # grid_x = round((cur_pos.x - map_origin.x) / map_res)
        # grid_y = round(((cur_pos.y - map_origin.y) / map_res))
        # #self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))

        # # binnum go from 1 to 3 so we can use uint8
        # # convert into 2D array using column order
        # odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))
        # # set current robot location to 0
        # odata[grid_y][grid_x] = 0
        # # create image from 2D array using PIL
        # img = Image.fromarray(odata)
        # # find center of image
        # i_centerx = iwidth/2
        # i_centery = iheight/2
        # # find how much to shift the image to move grid_x and grid_y to center of image
        # shift_x = round(grid_x - i_centerx)
        # shift_y = round(grid_y - i_centery)
        # # self.get_logger().info('Shift Y: %i Shift X: %i' % (shift_y, shift_x))

        # # pad image to move robot position to the center
        # # adapted from https://note.nkmk.me/en/python-pillow-add-margin-expand-canvas/ 
        # left = 0
        # right = 0
        # top = 0
        # bottom = 0
        # if shift_x > 0:
        #     # pad right margin
        #     right = 2 * shift_x
        # else:
        #     # pad left margin
        #     left = 2 * (-shift_x)
            
        # if shift_y > 0:
        #     # pad bottom margin
        #     bottom = 2 * shift_y
        # else:
        #     # pad top margin
        #     top = 2 * (-shift_y)
            
        # # create new image
        # new_width = iwidth + right + left
        # new_height = iheight + top + bottom
        # img_transformed = Image.new(img.mode, (new_width, new_height), map_bg_color)
        # img_transformed.paste(img, (left, top))

        # # rotate by 90 degrees so that the forward direction is at the top of the image
        # rotated = img_transformed.rotate(np.degrees(yaw)-90, expand=True, fillcolor=map_bg_color)

        # # show the image using grayscale map
        # # plt.imshow(img, cmap='gray', origin='lower')
        # # plt.imshow(img_transformed, cmap='gray', origin='lower')
        # #plt.imshow(rotated, cmap='gray', origin='lower')
        # #plt.draw_all()
        # # pause to make sure the plot gets created
        # #plt.pause(0.00000000001)

    def scan_callback(self, msg):
        self.laser_range = np.array(msg.ranges)
        #np.savetxt(scanfile, self.laser_range)
        self.laser_range[self.laser_range==0] = np.nan

    def push_button_callback(self, msg):
        print("returned from nfc_callback. Nothing is being done.")
        return
        print(f"push_button: {msg.data}")
        if(msg.data == "True"):
            self.is_drink_present = DRINK_PRESENT
        else:
            self.is_drink_present = DRINK_NOT_PRESENT
    def weight_sensor_callback(self, msg):
        #print(f"weight sensor: {msg.data}")
        #print("weight_sensor data ignored")
        #return
        if(msg.data >= 90000):
            self.is_drink_present = DRINK_PRESENT
        else:
            self.is_drink_present = DRINK_NOT_PRESENT
    def nfc_callback(self, msg):
        print("returned from nfc_callback. Nothing is being done.")
        return
        self.docking_phase_two()
    
    def rotatebot(self, rot_angle):
        print(f"in rotatebot, rot_angle: {rot_angle}")
        if(rot_angle == 0):
            return
        different_angle = 180

        target_angle = self.grid_angle + rot_angle
        different_angle = self.rotate_to_angle(target_angle, angular_velocity=1.0)
        #print(f"different_angle: {different_angle}")
        while(abs(different_angle) > 0.02):
            #print(f"different_angle: {different_angle}")
            different_angle = self.rotate_to_angle(target_angle, angular_velocity=0.1)
        
        self.first_run = True
        self.grid_angle += rot_angle
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        return

        print(f"in rotatebot, rot_angle: {rot_angle}")
        #rot_angle += 180 / math.pi * (self.second_yaw - self.first_yaw)
        if(rot_angle == 0):
            return
        different_angle = 180
        
        different_angle = self.rotate_unsafe(rot_angle, 4.0)
        while(abs(different_angle) > 0.25):
            print(f"different_angle: {different_angle}")
            different_angle = self.rotate_unsafe(-different_angle, 1.0)
        self.first_run = True
        rclpy.spin_once(self)
    
    def rotate_to_angle(self, target_angle, angular_velocity=rotatechange):
        twist = Twist()
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % current_yaw)
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        #target_yaw = math.radians(target_angle)
        target_yaw = target_angle
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % cmath.phase(c_target_yaw))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate

        twist.angular.z = c_change_dir * angular_velocity
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % current_yaw)
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        current_yaw = self.yaw
        return cmath.phase(c_target_yaw) - current_yaw

    def rotate_unsafe(self, rot_angle, angular_velocity=rotatechange):
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % current_yaw)
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + rot_angle
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % cmath.phase(c_target_yaw))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate

        twist.angular.z = c_change_dir * angular_velocity
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % current_yaw)
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)
        return cmath.phase(c_target_yaw) - current_yaw

    def calculate_angular_velocity_using_pid(self, current_yaw, target_yaw):
        error = target_yaw - current_yaw

        # if(error >= math.pi()):
        #     error -= 2*math.pi()
        # elif(error < math.pi()):
        #     error += 2*math.pi()

        k = kp * error
        i = ki * self.error_sum
        self.error_sum += error
        d = kd * (error - self.prev_error)
        self.prev_error = error
        angular_velocity = k + i + d

        #print(f"angular_velocity: {angular_velocity}\n\n")
        if(angular_velocity > 0.02):
            angular_velocity = 0.02
        if(angular_velocity < -0.02):
            angular_velocity = -0.02        

        return angular_velocity

    def calculate_wall_following_pid(self, current_distance, wall_distance):
        error = wall_distance - current_distance

        k = kp_w * error
        i = ki * self.w_error_sum
        self.w_error_sum += error
        d = kd * (error - self.w_prev_error)
        self.prev_error = error

        angular_velocity = k + i + d
        if(angular_velocity > 0.02):
            angular_velocity = 0.02
        if(angular_velocity < -0.02):
            angular_velocity = -0.02

        return angular_velocity

    def movebot(self, angular_velocity=0.0, target_yaw=0, current_yaw=0, speed=speedchange, direction=1):
        self.current_tick += 1
        if(self.current_tick != self.tick):
            self.current_tick = self.tick
            return
        self.movebot_unsafe(speed, direction, angular_velocity)
        return

    def movebot_unsafe(self, speed=speedchange, direction=1, angular_velocity=0):
        twist = Twist()
        twist.linear.x = speed * direction
        twist.angular.z = angular_velocity
        #time.sleep(1)
        self.publisher_.publish(twist)

    def move_distance_by_odom_then_varify_using_lidar(self, target_distance, 
    lidar_checking_index=0, lidar_checking_distance=0.3, direction=1, 
    is_using_lidar_to_check=False, distance_tolerance=0.05, ignore_lidar=False, check_can =False):
        #move forward if direction is one, speed is determined by choose_speed
        #print("inside move_distance by odom")
        
        initial_distance = self.linear_distance
        self.first_run = True
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                if self.laser_range.size != 0:
                    final_distance = self.linear_distance
                    remaining_distance = target_distance - final_distance + initial_distance
                    print(f"remaining_distance: {remaining_distance} = {target_distance} - {final_distance} + {initial_distance}")
                    speed = self.choose_speed(remaining_distance)
                    angular_velocity = self.calculate_angular_velocity_using_pid(current_yaw=self.yaw, target_yaw=self.grid_angle)
                    
                    lidar_distance = 0
                    for i in self.laser_range[left_angle-3: left_angle+4]:
                        lidar_distance += i

                    if(remaining_distance < 0):
                        self.stopbot()
                        break
                    elif(not ignore_lidar and self.laser_range[lidar_checking_index] <= lidar_checking_distance):
                        #print("lidar distance less than checking distance, stopping robot")
                        self.stopbot()
                        #reverse_op[self.table_number-1][] remaining_distance
                        break
                    else:
                        self.movebot(angular_velocity=angular_velocity, target_yaw=self.grid_angle, current_yaw=self.yaw, speed=speed, direction=direction)
        
            if(is_using_lidar_to_check == False):
                return

            while rclpy.ok():
                if self.laser_range.size != 0:
                    current = self.laser_range[lidar_checking_index] 
                    distance = current - lidar_checking_distance
                    #print(f"distance: {distance} = {current} - {lidar_checking_distance}")
                    speed = self.choose_speed(distance)
                    # if the list is not empty
                    if(abs(distance) < distance_tolerance):
                        self.stopbot()
                        return
                    else:
                        self.movebot(speed=speed, direction=distance/abs(distance))
                rclpy.spin_once(self)
            
        except Exception as e:
            print(e)
        finally:
            self.stopbot()

    def stopbot(self):
        #print('Stopping Robot')
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def choose_speed(self, distance):
        return 0.18
        if(distance <= 0.1):
            speed = 0.022
        elif(distance <= 0.2):
            speed = 0.10
        elif(distance <= 0.6):
            speed = 0.16
        elif(distance <= 1.0):
            speed = 0.18
        else:
            speed = 0.21

        return speed

    def deliver_phase_two(self):
        #additional procedure for table 6
        try:
            print("deliver_phase_two")
            #self.move_distance_by_odom_then_varify_using_lidar(0.94, front_angle, direction=1, is_using_lidar_to_check=True)
            while rclpy.ok():
                if self.laser_range.size != 0:
                    table_6_right_edge = 0
                    for i in self.laser_range[left_angle-4: left_angle+5]:
                        table_6_right_edge += i
                    table_6_right_edge /= 10
                    # table_6_right_edge += i in self.laser_range[left_angle-4:left_angle+5]
                    if(table_6_right_edge <= 1.7):
                        print("found table 6")
                        time.sleep(4)
                        self.stopbot() 
                        self.rotatebot(half_pi) 
                        break
                    print(f"table_6_right_edge: {table_6_right_edge}")
                    
                    speed = 0.08
                    self.movebot(speed=speed,direction=1)
                rclpy.spin_once(self)
           
            self.move_distance_by_odom_then_varify_using_lidar(1.92, front_angle, direction=1, is_using_lidar_to_check=False, lidar_checking_distance=0.25, ignore_lidar=False)


        except Exception as e:
            print(e)
        finally:
            self.stopbot()

    def deliver(self):
        print("inside deliver")
        try:
            vars = forward_op[self.table_number-1]
            i = 0
            while rclpy.ok():
                if(i >= len(vars)):
                    break
                self.first_run = True
                rclpy.spin_once(self)
                rclpy.spin_once(self)
                rclpy.spin_once(self)
                if(self.laser_range.size != 0):
                    self.move_distance_by_odom_then_varify_using_lidar(vars[i][0], 0, vars[i][1], 1, False, 0.05)
                    #if(vars[i][1] != 0):
                    self.rotatebot(vars[i][2])
                    self.first_run = True
                    rclpy.spin_once(self)
                    rclpy.spin_once(self)
                    rclpy.spin_once(self)
                    i += 1
                rclpy.spin_once(self)
            #special case for table 6
            if(self.table_number == 6):
                #input("table 6 debug")
                self.deliver_phase_two()
        except Exception as e:
            print(e)
        finally:
            self.stopbot()

    def docking_phase_one(self):
        try:
            self.first_run = True
            rclpy.spin_once(self)
            rclpy.spin_once(self)
            rclpy.spin_once(self)
            rclpy.spin_once(self)
            rclpy.spin_once(self)
            rclpy.spin_once(self)
            rclpy.spin_once(self)
            rclpy.spin_once(self)
            #returning to the dispenser
            vars = reverse_op[self.table_number-1]
            i = 0
            counter = 0
            while rclpy.ok():
                if(i>= len(vars)):
                    break
                if(self.laser_range.size != 0):
                    self.first_run = True
                    if(self.table_number-1 != 0 and i == len(vars) - 1):
                        self.move_distance_by_odom_then_varify_using_lidar(vars[i][0], back_angle, vars[i][1], -1, False, 0.05, False)
                    else:
                        self.move_distance_by_odom_then_varify_using_lidar(vars[i][0], back_angle, vars[i][1], -1, False, 0.05, False)
                    if(vars[i][2] != 0):
                        self.rotatebot(vars[i][2])
                        self.first_run = True
                        rclpy.spin_once(self)
                        rclpy.spin_once(self)
                        rclpy.spin_once(self)
                    i += 1
        except Exception as e:
            print(e)
        finally:
            self.stopbot()
        #self.docking_phase_two()

    def wall_following(self, target_distance, wall_distance, lidar_index, index_range=0):
        initial_distance = self.linear_distance
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.laser_range.size != 0:
                current_distance = 0
                for i in self.laser_range[lidar_index-index_range:lidar_index+index_range+1]:
                    current_distance += i
                current_distance /= (index_range*2+1)
                print(f"current distance: {current_distance}")
                final_distance = self.linear_distance
                remaining_distance = target_distance - final_distance + initial_distance
                #print(f"remaining_distance: {remaining_distance} = {target_distance} - {final_distance} + {initial_distance}")
                angular_velocity = self.calculate_wall_following_pid(current_distance=current_distance, wall_distance=0.5)
                speed = self.choose_speed(remaining_distance)
                # if the list is not empty
                if(remaining_distance < 0):
                    self.stopbot()
                    break
                else:
                    self.movebot(angular_velocity=angular_velocity, speed=speed, direction=-1)

        

    def docking_phase_two(self):
        try:
            #returning to the dispenser
            #measured distance: 14.6cm

            #self.move_distance_by_odom_then_varify_using_lidar(0.146, back_angle, direction=-1, is_using_lidar_to_check=True)
            if(self.table_number == 5):
                self.wall_following(2.44, 0.65, right_angle, 0)
                self.rotatebot(half_pi)
                self.move_distance_by_odom_then_varify_using_lidar(0.15, 0, 0.3, -1, False)
            else:
                self.wall_following(1.6, self.wall_distance, left_angle, 0)
        except Exception as e:
            print(e)
        finally:
            self.stopbot()

    def docking_phase_zero(self, duration=3.0):
        print("inside docking phase zero")
        
        self.first_run = True
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        
        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                speed = 0.16
                angular_velocity = 0.0
                if(self.is_drink_present == True):
                    start_time = time.time()
                    duration = time.time() - start_time
                    while(duration <= 0.3):
                        duration = time.time() - start_time
                        self.movebot(angular_velocity=angular_velocity, speed=speed, direction=-1)
                    break
                else:
                    self.movebot(angular_velocity=angular_velocity, speed=speed, direction=-1)
        except Exception as e:
            print("error in docking phase zero")
            print(e)
        finally:
            self.stopbot()

        return

    def UI(self):
        try:
            while(1):
                print("#"*20)
                self.table_input = input("#    Please Enter Table Number After Placing the Can    #\n")
                print("#"*20)
                print(f"#   \"{self.table_input}\" was received.    #")
                if(self.table_input.isdigit() == False):
                    print("#    Please Only Input Number from 1 to 6    #")
                    continue
                self.table_number = int(self.table_input)
                if(self.table_number > 6 or self.table_number < 1):
                    print("#    Please Only Input Number from 1 to 6    #")
                    continue
                break
        except Exception as e:
            print(e)
        finally:
            pass
        
    def procedure_loop(self):
        rclpy.spin_once(self)
        try:
            while(1):
                self.UI()
                self.docking_phase_zero()
                self.check_drink(exit_condition=DRINK_PRESENT)
                self.deliver()
                self.check_drink(exit_condition=DRINK_NOT_PRESENT)
                time.sleep(1)
                self.docking_phase_one()
        except Exception as e:
            print(e)
        finally:
            self.stopbot()

    def check_can(self):
        #true: can present; false: can not present
        #return is_drink_present

        #for debugging only
        print("inside check_can! For Debug Only")
        if(input("can present? [y|n] ") == "y"):
            return True
        else:
            return False

def main(args=None):

    rclpy.init(args=args)

    auto_nav = AutoNav()
    # while(1):
    #     rclpy.spin_once(auto_nav)
    #     auto_nav.rotatebot(-half_pi)
    #     time.sleep(0.5)
    #     input("Press Enter to continue")
    #auto_nav.table_number = 1
    #auto_nav.docking_phase_two()
    auto_nav.procedure_loop()
    #auto_nav.movebot()
    #rclpy.spin_once(auto_nav)
    #auto_nav.move_distance_by_odom_then_varify_using_lidar(1.27, 0, 0.3, -1)
    #input()
    #auto_nav.move_distance_by_odom_then_varify_using_lidar(1.27, 0, 0.3, -1)
    rclpy.spin(auto_nav)
    auto_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
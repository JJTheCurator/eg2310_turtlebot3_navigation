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

ROTATE_BIAS_DEGREE = 4.0
DRINK_PRESENT = True
DRINK_NOT_PRESENT = False



scanfile = 'lidar.txt'
mapfile = 'map.txt'

forward_op = [
    [(0.30, 0)],
    [(0.30, -90), (1.120, 0)],
    [(1.35, -90), (1.120, 0)],
    [(1.35, -90), (0.40, 0)],
    [(1.35, -90), (0.40, -90), (0.40, 90), (0.40, 90), (0.30, 0)],
    [(0.30, -90), (0.40, 90)],
]

reverse_op = [
    [(0.2, 0)],
    [(0.5, 90), (0.2, 0)],
    [(0.5, 90), (0.2, 0)],
    [(0.5, 90), (0.2, 0)],
    [(0.4, 90), (0.8, 90), (0.4, 90), (0.5, 90), (0.2, 90)],
    [(0.4, 90), (0.4, 90), (0.5, 90), (0.2, 90)],
]

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
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        self.one_meter_counter = 0

        self.push_button_subscription = self.create_subscription(
            String,
            'topic',
            self.push_button_callback,
            10,
        )
        self.push_button_subscription

        self.nfc_subscription = self.create_subscription(
            String,
            "nfc",
            self.nfc_callback,
            10,
        )
        self.nfc_subscription

        print("__init__ end")

        #UI logic
        self.table_number = 0
        self.table_input = "0"

        #navigation logic
        self.speed = 0.0
        self.first_run = True
        self.total_distance = 0.
        self.x = 0.0
        self.y = 0.0
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.linear_distance = 0.0

        #drink can logic
        self.is_drink_present = False

    def check_drink(self, exit_condition):
        print("in check_drink")
        while rclpy.ok():
            rclpy.spin_once(self) 
            if(self.is_drink_present == exit_condition):
                break
            time.sleep(0.3)
        if(exit_condition == DRINK_NOT_PRESENT):
            print("Drink is not present, proceeding.")
        else:
            print("Drink is present, proceeding")

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        if(self.first_run):
            self.previous_x = msg.pose.pose.position.x
            self.previous_y = msg.pose.pose.position.y
            self.first_run = False
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        d_increment = math.sqrt((x - self.previous_x) ** 2 +
                   (y - self.previous_y) ** 2)

        self.linear_distance += d_increment
        self.previous_x = msg.pose.pose.position.x
        self.previous_y = msg.pose.pose.position.y

        print(f"self.previous_x = {self.previous_x}")
        print(f"self.previous_y = {self.previous_y}")
        print(f"self.linear_distance = {self.linear_distance}")

    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # print to file
        #np.savetxt(mapfile, self.occdata)

    def scan_callback(self, msg):
        #print('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        #np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan
        print(self.laser_range)

    def push_button_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if(msg.data == "True"):
            self.is_drink_present = DRINK_PRESENT
        else:
            self.is_drink_present = DRINK_NOT_PRESENT
    def nfc_callback(self, msg):
        self.docking_phase_two()
    
    def rotatebot(self, rot_angle):
        if(rot_angle == 0):
            return
        # self.get_logger().info('In rotatebot')

        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
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

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)

    def rotatebot_with_one_time_distance_checking(self, rot_angle, checking_distance, checking_index):
        #checking the distance at checking_index inside lidar data against the checking_distance.
        angle = rot_angle*0.95
        remaining_angle = rot_angle-angle
        while(1):
            if(angle < 0.5):
                break
            self.rotatebot(angle)
            if(self.laser_range[checking_index] < checking_distance):
                angle = remaining_angle * 0.95
                remaining_angle = rot_angle - angle

    def movebot(self, speed=speedchange, direction=1):
        print(f"in movebot, speed = {speed}, self.speed = {speed}, direction = {direction}")
        if(speed == self.speed or speed == 0):
            print("speed == self.speed returning")
            return
        twist = Twist()
        twist.linear.x = speed * direction
        twist.angular.z = 0.0
        #time.sleep(1)
        self.publisher_.publish(twist)
        #self.speed = speed

    def move_distance_by_odom_then_varify_using_lidar(self, target_distance, lidar_checking_index=0, direction=1, is_using_lidar_to_check=False, distance_tolerance=0.05):
        #move forward if direction is one, speed is determined by choose_speed
        print("inside move_distance by odom")
        
        initial_distance = self.linear_distance
        try:
            while rclpy.ok():
                if self.laser_range.size != 0:
                    
                    final_distance = self.linear_distance
                    remaining_distance = target_distance - final_distance + initial_distance
                    print(f"remaining_distance: {remaining_distance} = {target_distance} - {final_distance} + {initial_distance}")
                    speed = self.choose_speed(remaining_distance)
                    # if the list is not empty
                    if(remaining_distance < 0 or self.laser_range[lidar_checking_index] <= 0.2):
                        self.stopbot()
                        break
                    else:
                        self.movebot(speed=speed, direction=direction)
                rclpy.spin_once(self)

            if(is_using_lidar_to_check == False):
                return

            while rclpy.ok():
                if self.laser_range.size != 0:
                    current = self.laser_range[lidar_checking_index] 
                    distance = current - target_distance
                    print(f"distance: {distance} = {current} - {target_distance}")
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
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def choose_speed(self, distance):
        if(distance <= 0.1):
            speed = 0.03
        elif(distance <= 0.3):
            speed = 0.2
        elif(distance <= 0.6):
            speed = 0.6
        elif(distance <= 1.0):
            speed = 0.8        
        else:
            speed = 1.0

        return speed

    def deliver_phase_two(self):
        #additional procedure for table 6
        try:
            self.move_distance_by_odom_then_varify_using_lidar(0.94, front_angle, direction=1, is_using_lidar_to_check=True)
            while rclpy.ok():
                if self.laser_range.size != 0:
                    table_6_right_edge = self.laser_range[left_angle]
                    if(table_6_right_edge <= 0.88):
                       self.stopbot() 
                       self.rotatebot(90+ROTATE_BIAS_DEGREE) 
                       break
                    print(f"table_6_right_edge: {table_6_right_edge}")
                    
                    #in case it fails to detect in the first run
                    #make it do shuttle run until it detects
                    front_distance = self.laser_range[front_angle]
                    direction = 1
                    if(front_distance <= 0.34):
                        direction = -1
                    if(front_distance >= 0.94):
                        direction = 1
                    speed = 0.1
                    self.movebot(speed,direction)
                rclpy.spin_once(self)
           
            self.move_distance_by_odom_then_varify_using_lidar(1.92, front_angle, direction=1, is_using_lidar_to_check=True)


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
                if self.laser_range.size != 0:
                    current = self.laser_range[front_angle] 
                    target = vars[i][0]
                    distance = current - target
                    print(f"distance: {distance} = {current} - {target}")
                    
                    speed = self.choose_speed(distance)
                    # if the list is not empty
                    if(distance < 0):
                        self.stopbot()
                        if(vars[i][1] != 0):
                            self.rotatebot(vars[i][1]+ROTATE_BIAS_DEGREE)
                        i += 1
                    else:
                        self.movebot(speed=speed)
                rclpy.spin_once(self)
            #special case for table 6
            if(self.table_number == 6):
                self.deliver_phase_two()
        except Exception as e:
            print(e)
        finally:
            self.stopbot()

    def docking_phase_one(self):
        try:
            #returning to the dispenser
            vars = reverse_op[self.table_number-1]
            i = 0
            while rclpy.ok():
                self.move_distance_by_odom_then_varify_using_lidar(0.146)
                rclpy.spin_once(self)
        except Exception as e:
            print(e)
        finally:
            self.stopbot()

    def docking_phase_two(self):
        try:
            #returning to the dispenser
            #measured distance: 14.6cm

            self.move_distance_by_odom_then_varify_using_lidar(0.146, back_angle, direction=-1, is_using_lidar_to_check=True)
        except Exception as e:
            print(e)
        finally:
            self.stopbot()

    def UI(self):
        try:
            while(1):
                print("#"*20)
                self.table_input = input("#    Please Enter Table Number After Placing the Can    #")
                print("#"*20)
                print(f"#   \"{self.table_input}\" was received.    #")
                if(self.table_input.isdigit() == False):
                    print("#    Only Input Number from 1 to 6    #")
                    continue
                self.table_number = int(self.table_input)
                if(self.table_number > 6 or self.table_number < 1):
                    print("#    Only Input Number from 1 to 6    #")
                    continue
                break
        except Exception as e:
            print(e)
        finally:
            self.stopbot()
        
    def procedure_loop(self):
        try:
            while(1):
                self.UI()
                #self.check_can()
                self.check_drink(exit_condition=DRINK_PRESENT)
                #self.deliver()
                self.check_can()
                self.check_drink(exit_condition=DRINK_NOT_PRESENT)
                self.docking_phase_one()
        except Exception as e:
            print(e)
        finally:
            self.stopbot()

    def check_can(self):
        #true: can present; false: can not present
        #return is_drink_present

        #for debugging only
        print("inside check_can!")
        if(input("can present? [y|n] ") == "y"):
            return True
        else:
            return False
    def test(self):
        while(1):
            self.movebot(0.1)
            time.sleep(0.5)
def main(args=None):

    rclpy.init(args=args)

    auto_nav = AutoNav()
    #auto_nav.test()
    auto_nav.procedure_loop()
    #auto_nav.movebot()
    #auto_nav.move_distance_by_odom_then_varify_using_lidar(0.30, 0)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #rclpy.spin(auto_nav)
    auto_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

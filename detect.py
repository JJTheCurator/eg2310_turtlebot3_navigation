import rclpy
import RPi.GPIO as GPIO
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np

servo_pin=20

GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)

class Scanner(Node):

    def __init__(self):
        super().__init__('scanner')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
    
    def servo_sweep(self, angle):
        duty = float(angle) / 18 + 2.5
        pwm.ChangeDutyCycle(duty)
        

    def listener_callback(self, msg):
        # create numpy array
        laser_range = np.array(msg.ranges)
        # replace 0's with nan
        laser_range[laser_range==0] = np.nan
        # find index with minimum value
        min_distance = np.nanmin(laser_range)
       
        if min_distance <= 1.0:
            running = True
            while running:
                self.servo_sweep(15)
                time.sleep(1)
                self.servo_sweep(165)
                time.sleep(1)
                running = False
        self.get_logger().info('Shortest distance: %f m' % min_distance)


def main(args=None):
    rclpy.init(args=args)
    pwm.start(5)
    pwm.ChangeFrequency(50)
    scanner = Scanner()

    rclpy.spin(scanner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scanner.destroy_node()
    rclpy.shutdown()
    pwm.stop()
    GPIO.cleanup()
    pwm.stop()

if __name__ == '__main__':
    main()
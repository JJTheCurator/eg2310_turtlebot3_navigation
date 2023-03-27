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

import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import RPi.GPIO as GPIO

from hx711 import HX711
hx = HX711(16, 12)

hx.set_reading_format("MSB", "MSB")

# HOW TO CALCULATE THE REFFERENCE UNIT
# To set the reference unit to 1. Put 1kg on your sensor or anything you have and know exactly how much it weights.
# In this case, 92 is 1 gram because, with 1 as a reference unit I got numbers near 0 without any weight
# and I got numbers around 184000 when I added 2kg. So, according to the rule of thirds:
# If 2000 grams is 184000 then 1000 grams is 184000 / 2000 = 92.
#hx.set_reference_unit(113)
#hx.set_reference_unit(referenceUnit)

hx.reset()
hx.tare()

GPIO.setmode(GPIO.BOARD)
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO_TRIG = 12
GPIO_ECHO = 16
GPIO.setup(GPIO_TRIG, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

class Publisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = self.check_push_button() 
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.check_ultrasonic()

    def check_push_button(self):
        print(GPIO.input(10))
        if(GPIO.input(10) == GPIO.HIGH):
            return "True"
        else:
            return "False"
    
    def check_ultrasonic(self):
        GPIO.output(GPIO_TRIG, True)
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIG, False)

        start_time = time.time()
        stop_time = time.time()

        while(GPIO.input(GPIO_ECHO) == 0):
            start_time = time.time()

        while(GPIO.input(GPIO_ECHO) == 1):
            stop_time = time.time()
        
        time_elapsed = stop_time - start_time
        distance = (time_elapsed * 34300) / 2

        print(distance)
        return distance

    def check_weight_sensor(self):

        weight = hx.get_weight(5)

        print(weight)
        hx.power_down()
        hx.power_up()
        return weight


def main(args=None):
    rclpy.init(args=args)

    publisher = Publisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

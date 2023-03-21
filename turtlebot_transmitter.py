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
import RPi.GPIO as GPIO
import time
from rclpy.node import Node

from std_msgs.msg import Bool
DRINK_PRESENT = 1
DRINK_NOT_PRESENT = 0

GPIO.setmode(GPIO.BOARD)
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

from mfrc522 import SimpleMFRC522
reader = SimpleMFRC522()



class TurtleBotTransmitter(Node):

    def __init__(self):
        super().__init__('turtleBot_transmitter')
        self.push_button_publisher = self.create_publisher(Bool, 'push_button', 10)
        self.nfc_publisher = self.create_publisher(Bool, 'nfc', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def check_loop(self):
        while(1):
            push_button_msg = Bool()
            self.push_button_publisher.publish(self.check_push_button())
            nfc_reader_msg = Bool()
            self.nfc_publisher.publish(self.check_nfc_reader())
            time.sleep(0.1)


    def check_push_button(self):
        if(GPIO.input(10) == GPIO.HIGH):
            return DRINK_PRESENT
        else:
            return DRINK_NOT_PRESENT

    def check_nfc_reader(self):
        id, text = reader.read()
        print("ID: %s\nText: %s" % (id,text))

        if(id != ""):
            return True
        else:
            return False

    def timer_callback(self):
        push_button_msg = Bool()
        nfc_msg = Bool()

        push_button_msg.data = self.check_push_button()
        nfc_msg.data = self.check_nfc_reader()

        self.push_button_publisher.publish(push_button_msg)
        self.nfc_publisher.publish(nfc_msg)


def main(args=None):
    rclpy.init(args=args)

    turtleBotTransmitter = TurtleBotTransmitter()

    rclpy.spin(turtleBotTransmitter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtleBotTransmitter.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
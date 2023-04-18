# eg2310_turtlebot3_navigation
ROS2 Auto_Nav Code for EG2310, Semester 2, AY21/22

Welcome to Group 4's repository for EG2310. This contains all the code for our Beta Gasoline robot.

Please follow our step-by-step guide to set up your system. We assume that you are using the exact same hardware as ours, such as our launcher, loading system, and electrical connections.

Preparation
To set up your laptop and TurtleBot, perform the following steps:

Follow the instructions here to set up your laptop and TurtleBot.
Set up i2c communication on the TurtleBot's Raspberry Pi by following these instructions.
Set up gpiozero and pigpio pin factory (to prevent servo spasms) using this guide.
Check out our /Hardware folder for the schematics of the PCBs we use to replicate our robot system. Important: We used a 5V supply instead of the 12V stated in our RPi HAT PCB, as 12V caused our motors to spin too fast and create oscillations in our robot.
File Allocation and Workspace Building
Clone our repository into your home directory:

bash
Copy code
git clone https://github.com/Magmanat/r2auto_nav.git
Then, move the entire hardware_bringup folder into your turtlebot3_ws using scp, build the workspace, and set up the ROS package on your laptop:

bash
Copy code
scp -r path_to_r2auto_nav/turtlebot3_ws/src/hardware_bringup ubuntu@(ip-address-of-pi):~/turtlebot3/src
ssh ubuntu@(ip-address-of-pi)
cd turtlebot3_ws
colcon build
bash
Copy code
cd path_to_r2auto_nav/colcon_ws
colcon build
After completing these steps, your system should be set up and ready to use our package.

System Check
First, we'll use the factory_test package to ensure everything is running correctly. SSH into the TurtleBot and start all the hardware publishers and subscribers.

For your TurtleBot:

In one terminal:

bash
Copy code
ssh ubuntu@(ip-address-of-pi)
roslaunch turtlebot3_bringup turtlebot3_robot.launch
In another terminal:

bash
Copy code
ssh ubuntu@(ip-address-of-pi)
ros2 launch hardware_bringup hardware.launch.py
Lastly, run the script on your laptop to verify the whole system is working:

For your Laptop:

In one terminal:

bash
Copy code
ros2 run auto_nav factory_test
Follow the instructions displayed in your terminal. If everything works as expected, your system is ready to go.

Running Auto_Nav in a Maze
Now, let's test the robot in an actual maze and see it complete its mission. Prepare two terminals for your TurtleBot and three terminals for your Laptop. Start all the hardware in the TurtleBot first, just as before.

For your TurtleBot:

In one terminal:

bash
Copy code
ssh ubuntu@(ip-address-of-pi)
roslaunch turtlebot3_bringup turtlebot3_robot.launch
In another terminal:

bash
Copy code
ssh ubuntu@(ip-address-of-pi)
ros2 launch hardware_b

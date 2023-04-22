# eg2310_turtlebot3_navigation
ROS2 Auto_Nav Code for EG2310, Semester 2, AY22/23

Welcome to Group 2's repository for EG2310. This contains all the code for our restaurant delivery robot.
In the repository, r2auto_nav.py is the navigation code to be ran on your computer. And turtlebot_transmitter.py is to be ran on your turtlebot

## Preparation
To set up your laptop and TurtleBot, perform the following steps:

Follow the instructions here to set up your laptop and TurtleBot https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
    Clone our repository into the ros2 source folder in your computer and turtlebot:

git clone https://github.com/JJTheCurator/eg2310_turtlebot3_navigation.git

#### On your computer:
    cd path_to_r2auto_nav/colcon_ws
    colcon build

#### On your turtlebot:
    cd turtlebot3_ws
    colcon build


## Factory Check
First, we'll use the factory_test package to ensure everything is running correctly. SSH into the TurtleBot and start all the hardware publishers and subscribers.

#### On your Turtlebot:

    ssh ubuntu@(ip-address-of-pi)
    roslaunch turtlebot3_bringup turtlebot3_robot.launch

#### On your Laptop:

    ros2 run auto_nav factory_test
  Follow the instructions displayed in your terminal. If everything works as expected, your system is ready to go.

## Actual Run
#### For your TurtleBot:
    ros2 run py_pubsub talker

#### On your computer:
    ros2 run r2auto_nav auto_nav
  Follow the instructions displayed in your terminal. If everything works as expected, your robot will deliver the drink and return to the dispenser.

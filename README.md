# Control-ROS1Noetic-WheelLeggedInfantrySimulation
Control-ROS1Noetic-WheelLeggedInfantrySimulation
# dependencies

ROS1_noetic

ros_control

Velocity_controller

position_controller


# Using
git clone to a workspace/src

catkin_make

source devel/setup.bash

sudo chmod +x FakeIMUPub.py CloseLoopController.py

roslaunch Wheel_Legged_Robot_description gazebo.launch 

roslaunch Wheel_Legged_Robot_description controller.launch 

gazebo is pausing, start it

# Control
origin is in the center of two leg drive motors, 
## Height
up is positive
 
rostopic pub /Wheel_Legged_Robot/Left_Height std_msgs/Float64 "data: 150.0" 
## Position
front is postive,

/Wheel_Legged_Robot/Left_Position
## Wheel speed
/Wheel_Legged_Robot/Left_Wheel_position_controller/command

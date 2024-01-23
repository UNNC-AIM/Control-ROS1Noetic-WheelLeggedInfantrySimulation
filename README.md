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
roslaunch Wheel_Legged_Robot_Simulation_Control simulation.launch 

# Control

rostopic pub /Wheel_Legged_Robot/Left_Height std_msgs/Float64 "data: 150.0" 

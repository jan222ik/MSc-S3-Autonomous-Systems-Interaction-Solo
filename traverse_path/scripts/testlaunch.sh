# Start World in Gazebo
roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch turtlebot3_gazebo turtlebot3_empty.launch

# Start SLAM
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping


# Start Odom, needs SLAM for calculations to map frame
roslaunch transformations_odom transformations_odom.launch


# Start Traverse Path Node
roslaunch traverse_path traverse_path.launch

# Drive Bot to better observe changes otherwise coordinates will stay the same
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

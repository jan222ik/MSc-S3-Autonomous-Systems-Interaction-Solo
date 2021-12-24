

# Start World in Gazebo
roslaunch turtlebot3_gazebo turtlebot3_house.launch

# Start SLAM and RVis
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

# Start Tagstore
roslaunch tagstore tagstore.launch


# Call service
rosservice call /tagstore-addtag 12 17

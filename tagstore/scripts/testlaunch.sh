

# Start World in Gazebo
roslaunch turtlebot3_gazebo turtlebot3_house.launch

# Start SLAM and RVis
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

# Start Tagstore
roslaunch tagstore tagstore.launch

# Reset RVis MarkerArray for tags
rosservice call /tagstore-reset-rvis-markers


# Call service
rosservice call /tagstore-addtag -- 150 150



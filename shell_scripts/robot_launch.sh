# ssh robot in Robot network
ssh pi@192.168.137.50


ssh pi@172.22.73.37

# ssh robot in Robot network v2
ssh pi@192.168.70.169

# ssh robot in labs
ssh pi@172.22.73.66

# Robot launch
roslaunch turtlebot3_bringup turtlebot3_robot.launch

# Robot remote (for robot state publisher, launch on remote PC)
roslaunch turtlebot3_bringup turtlebot3_remote.launch

# Raspicamera
roslaunch raspicam_node camerav2_1280x720.launch

# make sound
rostopic pub /sound turtlebot3_msgs/Sound '{value: 3}'

# tftree
rosrun rqt_tf_tree rqt_tf_tree

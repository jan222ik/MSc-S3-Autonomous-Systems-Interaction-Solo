
# Add to Bashrc:
settitle () {
  sleep 2
  export PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
  echo -ne '\033]0;'"$1"'\a'
}


# Start a World in Gazebo
settitle Gazebo & roslaunch turtlebot3_gazebo turtlebot3_house.launch
settitle Gazebo & roslaunch turtlebot3_gazebo turtlebot3_world.launch
settitle Gazebo & roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch


# Start SLAM
settitle SLAM & roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

# Start Odom, needs SLAM for calculations to map frame
settitle Odom & roslaunch transformations_odom transformations_odom.launch

# Start Tagstore
settitle TagStore & roslaunch tagstore tagstore.launch


# ABOVE or
settitle Combined & roslaunch plan_path sim-slam-odom.launch


settitle CostMap & roslaunch plan_path costmap.launch

# Start Plodding Node to provide Action Server
settitle Plodding & rosrun plodding turtle.py

# Start Traverse Path Node
settitle TraversePath & roslaunch traverse_path traverse_path.launch

# Start Plan Path Node
settitle PlanPath & rosrun plan_path planPath.py


# Drive Bot to better observe changes otherwise coordinates will stay the same
settitle Teleop & roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

# Reset RVis MarkerArray for tags
rosservice call /tagstore-reset-rvis-markers


# Call service
rosservice call /tagstore-addtag -- 150 150

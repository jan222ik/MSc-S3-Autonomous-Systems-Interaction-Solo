# MSc-S3-Autonomous-Systems-Interaction-Solo
ROS Packages for solving task related to course.

## Clone:
Execute clone in the source directory of your catkin workspace:
```sh
cd ~/catkin_ws/src
git clone https://github.com/jan222ik/MSc-S3-Autonomous-Systems-Interaction-Solo.git

git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git


rosdep install --from-paths src -i -y

git clone -b kinetic-devel https://github.com/ros-planning/navigation.git
sudo apt-get install libbullet-dev
sudo apt-get install libsdl-image1.2-dev
sudo apt-get install libsdl-dev

sudo apt-get install ros-kinetic-navigation
sudo apt-get install libnetpbm10-dev

```

WSL Location: ``\\wsl$\Ubuntu\home\$user\catkin_ws\src\MSc-S3-Autonomous-Systems-Interaction-Solo\``


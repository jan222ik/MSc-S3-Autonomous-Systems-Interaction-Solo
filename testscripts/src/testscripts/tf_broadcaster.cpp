#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
  // x y z transform = -0.032 0 0.172 -> taken from urdf joint link, check if correct!,
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.032, 0.0, 0.172)),
        ros::Time::now(),"base_link", "base_laser"));
    r.sleep();
  }
}
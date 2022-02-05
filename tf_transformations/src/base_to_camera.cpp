#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_base_laser_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
    /**
  // Quaternion from rpy:
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(r,p,y);
    myQuaternion=myQuaternion.normalize();
  **/

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        // TODO: check Quaternion rotation (none for camera_joint, some for camera link)
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.064, -0.065, 0.094)),
        ros::Time::now(),"base_link", "camera_link"));
    r.sleep();
  }
}
#! /usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from transformations_odom.msg import PoseInMap


class PoseConversions:

    def __init__(self):
        rospy.init_node('robot2map_conversion', log_level=rospy.DEBUG, anonymous=True)
        rospy.loginfo("PoseConversions: Startup")

        self.mapFrame = "map"
        self.robotFrame = "base_footprint"

        self.mapInfo = MapMetaData()
        self.mapInfo = rospy.wait_for_message("map", OccupancyGrid).info

        self.tfListener = tf.TransformListener()

        self.pubPoseInMap = rospy.Publisher("pose_in_map", PoseInMap, queue_size=10)

        self.subRobotPose = rospy.Subscriber('odom', Odometry, self._subNextPose)



        self.rate = rospy.Rate(2)
        rospy.spin()


    def _subNextPose(self, pose):
        """
        Callback of pose subscriber.
        Converts the current position of the robot in the map.
        """
        try:
            rospy.logdebug("PoseConversions: Next Pose")
            atTimeStamp = self.tfListener.getLatestCommonTime(self.mapFrame, self.robotFrame)
            pos, quad = self.tfListener.lookupTransform(self.mapFrame, self.robotFrame, atTimeStamp)

            resolution = self.mapInfo.resolution
            mapPose = PoseInMap()
            mapPose.x = (pos[0] - self.mapInfo.origin.position.x) / resolution
            mapPose.y = (pos[1] - self.mapInfo.origin.position.y) / resolution
            mapPose.header.seq = 0
            mapPose.header.stamp = rospy.Time.now()
            self.pubPoseInMap.publish(mapPose)
        except tf.Exception:
            rospy.logwarn("PoseConversions: Transform between /odom and /map is not ready")




def main():
    try:
        node = PoseConversions()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()


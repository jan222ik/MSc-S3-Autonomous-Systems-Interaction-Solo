#!/usr/bin/env python

from __future__ import print_function

import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData


class Localizer:
    def __init__(self):
        self.pose_estimate = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_estimate_callback)
        self.occupancy_grid = rospy.wait_for_message("/map", OccupancyGrid)
        pass

    def pose_estimate_callback(self, estimated_pose):
        rospy.loginfo(estimated_pose)

def main(args):
    loc = Localizer()
    rospy.init_node('localize_robot', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)

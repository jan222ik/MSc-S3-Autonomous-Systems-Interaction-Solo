#! /usr/bin/env python

import rospy
#import tf
#from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
#from std_msgs.msg import Header
#from transformations_odom.msg import PoseInMap

class TraversePath:

    def __init__(self):
        rospy.init_node('traverse_path', log_level=rospy.DEBUG, anonymous=True)
        rospy.loginfo("TraversePath: Startup")


def main():
    try:
        node = TraversePath()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

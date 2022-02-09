#! /usr/bin/env python

import rospy
import actionlib
import itertools
import numpy as np
#import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData # , Odometry
from std_msgs.msg import Header
#from visualization_msgs.msg import Marker, MarkerArray
#from transformations_odom.msg import PoseInMap
from traverse_path.msg import TraversalPoint, TraversePathAction, TraversePathActionGoal, TraversePathGoal, TraversePathActionFeedback
from actionlib_msgs.msg import GoalID
from tagstore.msg import Collab
from tagstore.srv import TagstoreAllTags
from plan_path.msg import PlanGoalAction, PlanGoalActionResult, PlanGoalResult, PlanGoalActionGoal, PlanGoalGoal, \
    PlanGoalActionFeedback
from transformations_odom.msg import PoseTF
from itertools import combinations
from itertools import product
from sys import stdout as out
import random, numpy, math, copy, matplotlib.pyplot as plt
from abc import ABCMeta, abstractmethod
from heapq import heappush, heappop, heapify

class Explorer:

    def __init__(self):
        self.isDone = False
        self.rate = rospy.Rate(10)

    def loop(self):
        while not rospy.is_shutdown() and not self.isDone:

            

            self.rate.sleep()

        rospy.logdebug("Explorer: Finished Scenario")

def main():
    try:
        node = Explorer()
        node.loop()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

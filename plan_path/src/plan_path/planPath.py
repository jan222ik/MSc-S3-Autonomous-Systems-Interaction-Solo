#! /usr/bin/env python

import rospy
import actionlib
#import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData # , Odometry
from std_msgs.msg import Header
#from visualization_msgs.msg import Marker, MarkerArray
#from transformations_odom.msg import PoseInMap
from traverse_path.msg import TraversalPoint, TraversePathAction, TraversePathActionGoal, TraversePathGoal, TraversePathActionFeedback
from actionlib_msgs.msg import GoalID

class PlanPath:

    def __init__(self):
        rospy.init_node('plan_path', log_level=rospy.DEBUG, anonymous=True)
        rospy.loginfo("PlanPath: Startup")

        rospy.logdebug("TraversePath: Await map message for MapInfo")
        self.mapInfo = MapMetaData()
        self.mapInfo = rospy.wait_for_message("map", OccupancyGrid).info
        rospy.logdebug("TraversePath: Await Action Client")
        client = actionlib.SimpleActionClient("traversal_action_server", TraversePathAction)
        rospy.logdebug("TraversePath: Finished Action Client")

        client.wait_for_server()

        # Creates a goal to send to the action server.
        l = [TraversalPoint(130, 220, 180), TraversalPoint(190, 210, 0), TraversalPoint(130, 220, 180)]
        g = TraversePathGoal()
        g.traversal_points = l



        # l = [TraversalPoint(130, 220, 180), TraversalPoint(190, 210, 0), TraversalPoint(130, 220, 180)]
        l = [TraversalPoint(190, 190, 180), TraversalPoint(170, 190, 0), TraversalPoint(160, 220, 180)]
        g = TraversePathGoal()
        g.traversal_points = l

        client.send_goal(g)


        client.wait_for_result()


        rospy.loginfo("Result: {}".format(client.get_result()))

def main():
    try:
        node = PlanPath()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

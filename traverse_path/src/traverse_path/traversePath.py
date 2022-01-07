#! /usr/bin/env python

import rospy
import actionlib
#import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData # , Odometry
#from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
#from transformations_odom.msg import PoseInMap
from traverse_path.msg import TraversePathAction, TraversePathActionResult, TraversePathActionFeedback

class TraversePath:

    def __init__(self):
        rospy.init_node('traverse_path', log_level=rospy.DEBUG, anonymous=True)
        rospy.loginfo("TraversePath: Startup")
        self.pubRvisMarkerArray = rospy.Publisher("traversal_points_marker_array", MarkerArray, queue_size = 1)
        rospy.logdebug("TraversePath: Await map message for MapInfo")
        self.mapInfo = MapMetaData()
        self.mapInfo = rospy.wait_for_message("map", OccupancyGrid).info
        rospy.logdebug("TraversePath: Starting Action Server")
        self.actionServer = actionlib.SimpleActionServer("traversal_action_server", TraversePathAction, execute_cb=self._nextActionGoal, auto_start = False)
        self.actionServer.start()
        rospy.logdebug("TraversePath: Started Action Server")

    def _nextActionGoal(self, data):
        self.traversalPoints = data.traversal_points
        self.externalCancel = False
        self.traversalIdx = 0

        self._publishRvisPoints(self.traversalPoints)

        self._scheduleNextGoal()

        result = TraversePathActionResult()
        feedback = TraversePathActionFeedback()

        while not rospy.is_shutdown() and self.externalCancel:
            if self.actionServer.is_preempt_requested():
                rospy.loginfo("TraversePath: Requested Cancel")
                self.externalCancel = True

            feedback.feedback.current_traversal_idx = self.traversalIdx
            self.actionServer.publish_feedback(feedback)

        result.result.traversal_has_finished.data = not self.traversalIdx < len(self.traversalPoints)
        self.actionServer.set_succeeded(result)




    def _scheduleNextGoal(self):
        if self.traversalIdx < len(self.traversalPoints):
            traversalPoint = self.traversalPoints[self.traversalIdx]
            rospy.loginfo("TraversePath: Next Goal: {}".format(traversalPoint))
            # TODO Actually Move
        else:
            self.externalCancel = True

    def _publishRvisPoints(self, points):
        """
        Publish Traversal Points to RVIS.
        Code copied from tagstore point published, changed colors
        """
        markers = MarkerArray()
        for point in points:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "rvismarker_traversal"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = point.x * self.mapInfo.resolution + self.mapInfo.origin.position.x
            marker.pose.position.y = point.y * self.mapInfo.resolution + self.mapInfo.origin.position.y
            marker.pose.position.z = 1
            markers.append(marker)

        idx = 0
        for m in markers:
            m.id = idx
            idx += 1

        self.pubRvisMarkerArray.publish(markers)
        rospy.sleep(0.01)


def main():
    try:
        node = TraversePath()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

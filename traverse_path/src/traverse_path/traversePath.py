#! /usr/bin/env python

import rospy
import actionlib
import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData # , Odometry
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
#from transformations_odom.msg import PoseInMap
from traverse_path.msg import TraversePathAction, TraversePathActionResult, TraversePathActionFeedback, TraversePathResult
from plodding.msg import PlodAction, PlodGoal


# noinspection DuplicatedCode
class TraversePath:

    def __init__(self):
        rospy.init_node('traverse_path', log_level=rospy.INFO, anonymous=True)
        rospy.loginfo("TraversePath: Startup")
        self.rate = rospy.Rate(200)
        self.pubRvisMarkerArray = rospy.Publisher("traversal_points_marker_array", MarkerArray, queue_size = 100)
        rospy.loginfo("TraversePath: Await map message for MapInfo")
        self.mapInfo = MapMetaData()
        self.mapInfo = rospy.wait_for_message("map", OccupancyGrid).info
        rospy.loginfo("TraversePath: Waiting for Plodding Action Server")
        self.ploddingClient = actionlib.SimpleActionClient("plodding_action_server", PlodAction)
        self.ploddingClient.wait_for_server()
        rospy.loginfo("TraversePath: Received Plodding Action Server")
        rospy.loginfo("TraversePath: Starting Action Server")
        self.actionServer = actionlib.SimpleActionServer("traversal_action_server", TraversePathAction, execute_cb=self._nextActionGoal, auto_start = False)
        self.actionServer.start()
        rospy.loginfo("TraversePath: Started Action Server")

    def thinTraversalPoints(self, points):
        isV = False
        isH = False
        last = points[0]
        n = [last]
        countEmpty = 0

        for idx in range(1, len(points) - 1):
            it = points[idx]
            rospy.loginfo("Traverse > Thin Points: Idx:{} isH:{} isV{} dx:{} dy:{}".format(idx, isH, isV, abs(last.x - it.x), abs(last.y - it.y)))
            dx = abs(last.x - it.x) == 1
            dy = abs(last.y - it.y) == 1

            add = False
            if dx and dy:
                if isH and isV:
                    pass
                else:
                    add = True
            else:
                if isV:
                    if dy:
                        pass
                    else:
                        add = True
                elif isH:
                    if dx:
                        pass
                    else:
                        add = True

            if add or countEmpty > 3:
                n.append(it)
                countEmpty = 0
            else:
                countEmpty += 1

            isH = dx
            isV = dy
            last = it

        n.append(points[len(points) - 1])
        rospy.loginfo("Traverse > Thin Points: Before:{} After:{}".format(len(points), len(n)))
        return n




    def _nextActionGoal(self, data):
        self.traversalPoints = self.thinTraversalPoints(data.traversal_points)
        self.externalCancel = False
        self.traversalIdx = 0



        self._scheduleNextGoal()

        result = TraversePathActionResult()
        result.result = TraversePathResult()
        result.result.traversal_has_finished = Bool()
        feedback = TraversePathActionFeedback()

        while not rospy.is_shutdown() and not self.externalCancel:
            if self.actionServer.is_preempt_requested():
                rospy.loginfo("TraversePath: Requested Cancel")
                self.externalCancel = True
                self.ploddingClient.cancel_all_goals()

            feedback.feedback.current_traversal_idx = self.traversalIdx
            self.actionServer.publish_feedback(feedback.feedback)
            self.rate.sleep()

        result.result.traversal_has_finished.data = not self.traversalIdx < len(self.traversalPoints)
        self.actionServer.set_succeeded(result.result)


    def _scheduleNextGoal(self):
        if self.traversalIdx < len(self.traversalPoints) and not self.externalCancel:
            traversalPoint = self.traversalPoints[self.traversalIdx]
            rospy.loginfo("TraversePath: Next Goal: {}".format(traversalPoint))
            goalPose = self.toGoalPose(traversalPoint, self.mapInfo)
            goal = PlodGoal()
            goal.target = goalPose
            goal.waitAfter = Bool(self.traversalIdx + 1 == len(self.traversalPoints))
            self._publishRvisPoints(self.traversalPoints)
            self.ploddingClient.send_goal(goal, done_cb=self._onDone)

        else:
            self.externalCancel = True

    def _onDone(self, goalID, result):
        rospy.loginfo("DONE GOAL {}: {}".format(goalID, result))
        self.traversalIdx+=1
        self._scheduleNextGoal()

    @staticmethod
    def toGoalPose(traversalPoint, mapInfo):
        pose = Pose()
        pose.position.x = (traversalPoint.x * mapInfo.resolution) + mapInfo.origin.position.x
        pose.position.y = (traversalPoint.y * mapInfo.resolution) + mapInfo.origin.position.y
        quaternion = tf.transformations.quaternion_from_euler(0, 0, traversalPoint.angle)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose

    def _publishRvisPoints(self, points):
        """
        Publish Traversal Points to RVIS.
        Code copied from tagstore point published, changed colors
        """
        markers = MarkerArray()
        for idx, point in enumerate(points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "rvismarker_traversal"
            marker.type = marker.ARROW
            marker.action = marker.ADD
            scale = 0.1
            marker.scale.x = scale
            marker.scale.y = scale / 2
            marker.scale.z = scale
            isCurrent = self.traversalIdx == idx
            marker.color.r = 0 if isCurrent else 1.0
            marker.color.g = 0
            marker.color.b = 1.0
            marker.color.a = 1.0
            quaternion = tf.transformations.quaternion_from_euler(0, 0, point.angle)
            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]
            marker.pose.position.x = point.x * self.mapInfo.resolution + self.mapInfo.origin.position.x
            marker.pose.position.y = point.y * self.mapInfo.resolution + self.mapInfo.origin.position.y
            marker.pose.position.z = 1
            markers.markers.append(marker)

        idx = 0
        for m in markers.markers:
            m.id = idx
            idx += 1

        self.pubRvisMarkerArray.publish(markers)
        rospy.sleep(0.01)


def main():
    try:
        node = TraversePath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

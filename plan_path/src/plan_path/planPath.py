#! /usr/bin/env python

import rospy
import actionlib
import numpy as np
# import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData  # , Odometry
from std_msgs.msg import Header, Bool
# from visualization_msgs.msg import Marker, MarkerArray
from transformations_odom.msg import PoseTF
from traverse_path.msg import TraversalPoint, TraversePathAction, TraversePathActionGoal, TraversePathGoal, \
    TraversePathActionFeedback
from plan_path.msg import PlanGoalAction, PlanGoalActionResult, PlanGoalResult, PlanGoalActionGoal, PlanGoalGoal, \
    PlanGoalActionFeedback
from actionlib_msgs.msg import GoalID

from abc import ABCMeta, abstractmethod
from heapq import heappush, heappop, heapify


def add(a, b):
    return a[0] + b[0], a[1] + b[1]


N = (0, 1)
S = (0, -1)
E = (1, 0)
W = (-1, 0)
NW = add(N, W)
NE = add(N, E)
SW = add(S, W)
SE = add(S, E)

cardinals = [N, NE, E, SE, S, SW, W, NW]


class PlanPath:

    def __init__(self):
        rospy.init_node('plan_path', log_level=rospy.DEBUG, anonymous=True)
        rospy.loginfo("PlanPath: Startup")
        self.neighbourThreshold = 20
        self.rate = rospy.Rate(5)
        rospy.logdebug("PlanPath: Await map message for MapInfo")
        self.mapInfo = MapMetaData()
        self.mapInfo = rospy.wait_for_message("map", OccupancyGrid).info
        rospy.logdebug("PlanPath: Await Action Client")
        self.client = actionlib.SimpleActionClient("traversal_action_server", TraversePathAction)
        self.client.wait_for_server()
        rospy.logdebug("PlanPath: Finished Action Client")

        rospy.logdebug("PlanPath: Starting Action Server")
        self.actionServer = actionlib.SimpleActionServer("plan_path_action_server", PlanGoalAction,
                                                         execute_cb=self._nextActionGoal, auto_start=False)
        self.actionServer.start()
        rospy.logdebug("PlanPath: Started Action Server")
        self.isDone = False

        rospy.logdebug("PlanPath: Enter Spin")
        rospy.spin()

    def _nextActionGoal(self, data):
        goal = (data.x, data.y)
        rospy.logdebug("PlanPath: Next Goal {}".format(goal))
        self.externalCancel = False
        self.isDone = False

        self.planAndExecute(goal)

        result = PlanGoalResult()
        result.has_finished = Bool()

        while not rospy.is_shutdown() and not self.externalCancel and not self.isDone:
            if self.actionServer.is_preempt_requested():
                rospy.loginfo("PlanPath: Requested Cancel")
                self.externalCancel = True
                self.client.cancel_all_goals()
            self.rate.sleep()

        result.has_finished.data = self.isDone
        self.actionServer.set_succeeded(result)

    def planAndExecute(self, goal):
        rospy.logdebug("PlanPath: Plan Path")
        poseTF = rospy.wait_for_message("pose_tf", PoseTF)
        start = (poseTF.mapPose.x, poseTF.mapPose.y)
        tps = self.createTraversalPoints(
            startPosePair=start,
            goalPosePair=goal
        )
        g = TraversePathGoal()
        g.traversal_points = tps
        rospy.logdebug("PlanPath: Execute path")
        self.client.send_goal(g, done_cb=self._onDone)

    def _onDone(self, goalId, result):
        rospy.loginfo("PlanPath: Traversal Result: {} for goalID {}".format(result, goalId))
        self.isDone = result.traversal_has_finished.data
        if not self.isDone:
            self.externalCancel = True

    def createTraversalPoints(self, startPosePair, goalPosePair):
        rospy.logdebug("PlanPath: Create Traversal Points")
        costmapMsg = rospy.wait_for_message("/costmap_node/costmap/costmap", OccupancyGrid)
        costmap = np.array(costmapMsg.data, dtype=np.int8).reshape(costmapMsg.info.height, costmapMsg.info.width)
        rospy.logdebug("PlanPath: From: {} To: {}".format(startPosePair, goalPosePair))
        path = list(find_path(
            start=startPosePair,
            goal=goalPosePair,
            neighbors_fnct=lambda cell: self.neighborsForCell(costmap, cell),
            reversePath=False,
            heuristic_cost_estimate_fnct=self.euclideanDistanceBetweenCells,
            distance_between_fnct= lambda a, b: self.costBetween(a, b, costmap, costmapMsg.info),
            is_goal_reached_fnct=lambda a, b: abs(a[0] - b[0]) < 0.5 and abs(a[1] - b[1]) < 0.5
        ))
        traversalPoints = []
        nextPoint = None
        for point in path[::-1]:
            if nextPoint is None:
                angle = 0
            else:
                vec = (
                    point[0] - nextPoint[0],
                    point[1] - nextPoint[1],
                )
                angle = self.angle_between([1,0], vec)
            nextPoint = point
            traversalPoints.append(TraversalPoint(point[0], point[1], angle))

        waypoints = traversalPoints[::-1]
        if len(waypoints) > 1:
            waypoints.pop(0)
        return waypoints

    def costBetween(self, a, b, costmap, info):
        cost = 1 + max(0, self.extractCostAt(costmap, info, b[0], b[1]))
        return cost

    @staticmethod
    def extractCostAt(costmap, info, x, y):
        if -1 < x < info.width and -1 < y < info.height:
            # data comes in row-major order http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
            # first index is the row, second index the column
            return costmap[y][x]
        else:
            IndexError(
                "Coordinates out of gridmap, x: {}, y: {} must be in between: [0, {}], [0, {}]".format(
                    x, y, info.height, info.width))
            return None



    def neighborsForCell(self, cm, cell):
        l = []
        for direction in cardinals:
            target = add(cell, direction)
            try:
                if cm[target[0]][target[1]] < self.neighbourThreshold:
                    l.append(target)
            except IndexError:
                pass
                # rospy.logwarn("PlanPath: Costmap index access outside of bounds: {}".format(target))
        return l

    @staticmethod
    def angle_between(p1, p2):
        #ang1 = np.arctan2(*p1[::-1])
        #ang2 = np.arctan2(*p2[::-1])
        #return np.rad2deg((ang1 - ang2) % (2 * np.pi))
        a = np.array(p1)
        b = np.array(p2)

        inner = np.inner(a, b)
        norms = np.linalg.norm(a) * np.linalg.norm(b)

        cos = inner / norms
        rad = np.arccos(np.clip(cos, -1.0, 1.0))
        return cos

    def neighborsForCell(self, costmap, cell):
        l = []
        for direction in cardinals:
            target = add(cell, direction)
            try:
                if costmap[target[0]][target[1]] < self.neighbourThreshold:
                    l.append(target)
            except IndexError:
                pass
                # rospy.logwarn("PlanPath: Costmap index access outside of bounds: {}".format(target))
        return l

    @staticmethod
    def euclideanDistanceBetweenCells(a, b):
        x = np.power(b[0] - a[0], 2)
        y = np.power(b[1] - a[1], 2)
        return np.sqrt(np.sum([x, y]))

#### Not our ASTAR Impl

# -*- coding: utf-8 -*-
""" generic A-Star path searching algorithm """

__author__ = "Julien Rialland"
__copyright__ = "Copyright 2012-2017, J.Rialland"
__license__ = "BSD"
__version__ = "0.93"
__maintainer__ = __author__
__email__ = ''.join(map(chr, [106, 117, 108, 105, 101, 110, 46, 114, 105,
                              97, 108, 108, 97, 110, 100, 64, 103, 109, 97, 105, 108, 46, 99, 111, 109]))
__status__ = "Production"

Infinite = float('inf')


class AStar:
    __metaclass__ = ABCMeta
    __slots__ = ()

    class SearchNode:
        __slots__ = ('data', 'gscore', 'fscore',
                     'closed', 'came_from', 'out_openset')

        def __init__(self, data, gscore=Infinite, fscore=Infinite):
            self.data = data
            self.gscore = gscore
            self.fscore = fscore
            self.closed = False
            self.out_openset = True
            self.came_from = None

        def __lt__(self, b):
            return self.fscore < b.fscore

    class SearchNodeDict(dict):

        def __missing__(self, k):
            v = AStar.SearchNode(k)
            self.__setitem__(k, v)
            return v

    @abstractmethod
    def heuristic_cost_estimate(self, current, goal):
        """Computes the estimated (rough) distance between a node and the goal, this method must be implemented in a subclass. The second parameter is always the goal."""
        raise NotImplementedError

    @abstractmethod
    def distance_between(self, n1, n2):
        """Gives the real distance between two adjacent nodes n1 and n2 (i.e n2 belongs to the list of n1's neighbors).
           n2 is guaranteed to belong to the list returned by the call to neighbors(n1).
           This method must be implemented in a subclass."""
        raise NotImplementedError

    @abstractmethod
    def neighbors(self, node):
        """For a given node, returns (or yields) the list of its neighbors. this method must be implemented in a subclass"""
        raise NotImplementedError

    def is_goal_reached(self, current, goal):
        """ returns true when we can consider that 'current' is the goal"""
        return current == goal

    def reconstruct_path(self, last, reversePath=False):
        def _gen():
            current = last
            while current:
                yield current.data
                current = current.came_from

        if reversePath:
            return _gen()
        else:
            return reversed(list(_gen()))

    def astar(self, start, goal, reversePath=False):
        if self.is_goal_reached(start, goal):
            return [start]
        searchNodes = AStar.SearchNodeDict()
        startNode = searchNodes[start] = AStar.SearchNode(
            start, gscore=.0, fscore=self.heuristic_cost_estimate(start, goal))
        openSet = []
        heappush(openSet, startNode)
        while openSet:
            current = heappop(openSet)
            if self.is_goal_reached(current.data, goal):
                return self.reconstruct_path(current, reversePath)
            current.out_openset = True
            current.closed = True
            for neighbor in map(lambda n: searchNodes[n], self.neighbors(current.data)):
                if neighbor.closed:
                    continue
                tentative_gscore = current.gscore + \
                                   self.distance_between(current.data, neighbor.data)
                if tentative_gscore >= neighbor.gscore:
                    continue
                neighbor.came_from = current
                neighbor.gscore = tentative_gscore
                neighbor.fscore = tentative_gscore + \
                                  self.heuristic_cost_estimate(neighbor.data, goal)
                if neighbor.out_openset:
                    neighbor.out_openset = False
                    heappush(openSet, neighbor)
                else:
                    # re-add the node in order to re-sort the heap
                    openSet.remove(neighbor)
                    heappush(openSet, neighbor)
        return None


def find_path(
        start,
        goal,
        neighbors_fnct,
        reversePath=False,
        heuristic_cost_estimate_fnct=lambda a, b: Infinite,
        distance_between_fnct=lambda a, b: 1.0,
        is_goal_reached_fnct=lambda a, b: a == b
):
    """A non-class version of the path finding algorithm"""

    class FindPath(AStar):

        def heuristic_cost_estimate(self, current, goal):
            return heuristic_cost_estimate_fnct(current, goal)

        def distance_between(self, n1, n2):
            return distance_between_fnct(n1, n2)

        def neighbors(self, node):
            return neighbors_fnct(node)

        def is_goal_reached(self, current, goal):
            return is_goal_reached_fnct(current, goal)

    return FindPath().astar(start, goal, reversePath)


def main():
    try:
        node = PlanPath()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()

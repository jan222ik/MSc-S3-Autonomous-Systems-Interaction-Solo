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

from abc import ABCMeta, abstractmethod
from heapq import heappush, heappop, heapify

def add(a, b):
    return a[0] + b[0], a[1] + b[1]


N = ( 0,  1)
S = ( 0, -1)
E = ( 1,  0)
W = (-1,  0)
NW = add(N, W)
NE = add(N, E)
SW = add(S, W)
SE = add(S, E)

cardinals=[N,NE,E,SE,S,SW,W,NW]


class FieldTrip:

    def __init__(self):
        rospy.init_node('field_trip', log_level=rospy.DEBUG, anonymous=True)
        rospy.loginfo("FieldTrip: Startup")
        rospy.logdebug("FieldTrip: Await tagstore 'tagstore-alltags' service proxy")
        rospy.wait_for_service("tagstore-alltags")
        self.taglist = rospy.ServiceProxy("tagstore-alltags", TagstoreAllTags)()
        rospy.logdebug("FieldTrip: Await map message for MapInfo")
        self.mapInfo = MapMetaData()
        self.mapInfo = rospy.wait_for_message("map", OccupancyGrid).info
        rospy.logdebug("FieldTrip: Await calculation for all paths between Tags to finish")
        self.pathLookupMap = self.calculatePathsBetweenTags(self.taglist)
        rospy.logdebug("FieldTrip: Finished Path between Tag Calculation")

        rospy.logdebug("FieldTrip: Await Action Client for traversal")
        client = actionlib.SimpleActionClient("traversal_action_server", TraversePathAction)
        rospy.logdebug("FieldTrip: Finished Action Client")

        client.wait_for_server()

        self.subTagReached = rospy.Subscriber("collab-tag-approach", Collab, self.subTagReached)


    def subTagReached(self, data):
        if data.hasReached.data:
            if data.tagID in self.taglist:
                # TODO Throw tag out of list to visit and recalculate path
                pass


    def plan_global(self, startMapPose):
        """
            Gives a global plan in which order to visit the tags.
        """
        start = (startMapPose.x, startMapPose.y)
        minTag = None
        minDist = 100000000
        for tag in self.taglist:
            distance_between_cells = self.euclideanDistanceBetweenCells(start, (tag.x, tag.y))
            # TODO determine if wall is direct way
            if distance_between_cells < minDist:
                minDist = distance_between_cells
                minTag = tag

        


        pass

    def calculatePathsBetweenTags(self, tagList):
        """
            Calculates all paths between 2 given tags
        """
        paths = [[]]
        for p in list(itertools.combinations(range(len(tagList)), 2)):
            start = tagList[p[0]]
            goal = tagList[p[1]]
            if paths[start.global_id][goal.global_id] is None or paths[goal.global_id][start.global_id] is None:
                path = find_path(
                    start=(start.x, start.y),
                    goal=(goal.x, goal.y),
                    neighbors_fnct=lambda cell: self.neighborsForCell(cell),
                    reversePath=False,
                    heuristic_cost_estimate_fnct=self.euclideanDistanceBetweenCells,
                    distance_between_fnct=lambda a, b: 1,
                    is_goal_reached_fnct=lambda a, b: a == b
                )
                cost = len(path)
                paths[start.global_id][goal.global_id]=(start, goal, cost, path)
                paths[goal.global_id][start.global_id]=(goal, start, cost, path[::-1])
        return paths



    def neighborsForCell(self, cell):
        l = []
        for direction in cardinals:
            target = add(cell, direction)
            # TODO check if target exist and can be driven on
            l.append(target)
        return l

    @staticmethod
    def euclideanDistanceBetweenCells(a, b):
        return np.linalg.norm(a-b)





def main():
    try:
        node = FieldTrip()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()


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




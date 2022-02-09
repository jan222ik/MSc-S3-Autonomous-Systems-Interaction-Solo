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
        self.neighbourThreshold = 80
        rospy.logdebug("FieldTrip: Await tagstore 'tagstore_alltags' service proxy")
        rospy.wait_for_service("tagstore_alltags")
        self.taglist = rospy.ServiceProxy("tagstore_alltags", TagstoreAllTags)().tags
        rospy.logdebug("FieldTrip: {} Tags: {}".format(len(self.taglist), self.taglist))
        self.distlist = []
        self.costmap = []
        for i in range(len(self.taglist)):
            self.costmap.append([])
            for j in range(len(self.taglist)):
                self.costmap[i].append(None)

        rospy.logdebug("FieldTrip: Await map message for MapInfo")
        self.mapInfo = MapMetaData()
        self.mapInfo = rospy.wait_for_message("map", OccupancyGrid).info
        rospy.logdebug("FieldTrip: Await calculation for all paths between Tags to finish")
        self.pathLookupMap = self.calculatePathsBetweenTags(self.taglist)
        rospy.logdebug("FieldTrip: Finished Path between Tag Calculation")
        rospy.logdebug("FieldTrip: Await current map pose")
        pose = rospy.wait_for_message("pose_tf", PoseTF).mapPose
        rospy.logdebug("FieldTrip: Await calculation of Traveling Sales Man to finish")
        self.tsp = self.plan_global(startMapPose=pose)
        print(self.tsp)
        rospy.logdebug("FieldTrip: Finished Traveling Sales Man Calculation")

        rospy.logdebug("FieldTrip: Await Action Client 'plan_path_action_server'")
        self.client = actionlib.SimpleActionClient("plan_path_action_server", PlanGoalAction)
        self.client.wait_for_server()
        rospy.logdebug("FieldTrip: Finished Action Client")


        self.subTagReached = rospy.Subscriber("collab_tag_approach", Collab, self.subTagReached)
        self.pubTagReached = rospy.Publisher("collab_tag_approach", Collab, queue_size=50)

        self.visited = [False for i in range(len(self.taglist))]
        self.current = 0
        self.nextNavTarget()

        self.isDone = False
        self.rate = rospy.Rate(1)

        self.executing = False
        while not rospy.is_shutdown() and not self.isDone:
            loopLog = "./."
            if self.visited[self.current]:
                loopLog = "IDX: {} already visited.".format(self.current)
                self.current = (self.current +  1) % len(self.visited)
                self.executing = False
                if all(self.visited):
                    self.isDone = True
            elif not self.executing:
                loopLog = "Next Target"
                self.executing = True
                self.nextNavTarget()

            rospy.logdebug("FieldTrip: {} > Idx: {} > TagID: {} > Loop Msg: {}".format(self.visited, self.current, self.tsp[self.current].global_id, loopLog))
            self.rate.sleep()

        rospy.logdebug("FieldTrip: Finished > Publish Goal to Stop Robot")
        # Publish another goal to stop the robot in case the last goal gets canceled
        self.client.cancel_all_goals()
        # pose = rospy.wait_for_message("pose_tf", PoseTF).mapPose
        # self.client.send_goal_and_wait(goal= PlanGoalGoal(x=pose.x, y=pose.y))
        rospy.logdebug("FieldTrip: Finished Scenario")


    def nextNavTarget(self):
        rospy.logdebug("FieldTrip: Get Nav Target")
        targetTag = self.tsp[self.current]
        goalData = PlanGoalGoal(x=targetTag.x, y=targetTag.y)
        rospy.logdebug("FieldTrip: Send Nav Target {}".format(targetTag))
        self.client.send_goal(
            goal=goalData,
            done_cb=self.onDoneGoal
        )

    def onDoneGoal(self, goalId, data):
        self.visited[self.current] = True


    def subTagReached(self, data):
        rospy.logdebug("FieldTrip: Reached Msg: {}".format(data))
        if data.hasReached.data:
            rospy.logdebug("FieldTrip: Tag was reached")
            idx = 0
            for tag in self.tsp:
                if data.tag.global_id  == tag.global_id:
                    if not self.visited[idx]:
                        rospy.logdebug("FieldTrip: Tag set visited to True")
                        self.visited[idx] = True
                        if idx == self.current:
                            rospy.logdebug("FieldTrip: Active Goal was reached by other robot or marked from CLI")
                            # self.onDoneGoal(None, None)
                    break
                else:
                    idx += 1


    def plan_global(self, startMapPose):
        """
            Gives a global plan in which order to visit the tags.
            @rtype: list
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

        orderOfTags = self.tspSolve()
        before, after = [], []
        found = False
        for t in orderOfTags:
            if t.global_id == minTag.global_id:
                found=True
            elif found:
                after.append(t)
            else:
                before.append(t)

        return [minTag] + after + before

    def calculatePathsBetweenTags(self, tagList):
        """
            Calculates all paths between 2 given tags
        """
        paths = [[None for col in range(len(tagList))] for row in range(len(tagList))]
        print paths
        costmapMsg = rospy.wait_for_message("/costmap_node/costmap/costmap", OccupancyGrid)
        costmap = np.array(costmapMsg.data, dtype=np.int8).reshape(costmapMsg.info.height,costmapMsg.info.width)
        for p in list(itertools.combinations(range(len(tagList)), 2)):
            start = tagList[p[0]]
            goal = tagList[p[1]]
            if paths[start.global_id][goal.global_id] is None or paths[goal.global_id][start.global_id] is None:
                path = list(find_path(
                    start=(start.x, start.y),
                    goal=(goal.x, goal.y),
                    neighbors_fnct=lambda cell: self.neighborsForCell(costmap, cell),
                    reversePath=False,
                    heuristic_cost_estimate_fnct=self.euclideanDistanceBetweenCells,
                    distance_between_fnct=lambda a, b: 1 + self.extractCostAt(costmap, costmapMsg.info, b[0], b[1]),
                    is_goal_reached_fnct=lambda a, b: abs(a[0] - b[0]) < 0.5 and abs(a[1] - b[1]) < 0.5
                ))
                print("Path {}".format(path))
                cost = len(path)
                paths[start.global_id][goal.global_id]=(start, goal, cost, path)
                paths[goal.global_id][start.global_id]=(goal, start, cost, path[::-1])
                self.distlist.append((p[0], p[1], cost))
                self.costmap[p[0]][p[1]] = cost
                self.costmap[p[1]][p[0]] = cost
        return paths


    @staticmethod
    def extractCostAt(costmap, info, x, y):
        if -1 < x < info.width and -1 < y < info.height:
            # data comes in row-major order http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
            # first index is the row, second index the column
            return costmap[y][x]
        else:
            raise IndexError(
                "Coordinates out of gridmap, x: {}, y: {} must be in between: [0, {}], [0, {}]".format(
                    x, y, info.height, info.width))



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
    def euclideanDistanceBetweenCells(a, b):
        x = np.power(b[0] - a[0], 2)
        y = np.power(b[1] - a[1], 2)
        return np.sqrt(np.sum([x, y]))

    def cost(self, c0, c1):
        return self.costmap[c0.global_id][c1.global_id]

    def tspSolve(self):
        tags = self.taglist
        N = len(tags)
        tour = random.sample(range(N),N)
        for temperature in numpy.logspace(0,5,num=100000)[::-1]:
            [i,j] = sorted(random.sample(range(N),2))
            newTour =  tour[:i] + tour[j:j+1] +  tour[i+1:j] + tour[i:i+1] + tour[j+1:]
            oldDistances = sum([
                self.cost(tags[tour[(k + 1) % N]], tags[tour[k % N]]) for k in[j, j - 1, i, i - 1]]
            )
            newDistances = sum([
                self.cost(tags[newTour[(k + 1) % N]], tags[newTour[k % N]]) for k in[j, j - 1, i, i - 1]]
            )
            if math.exp((oldDistances - newDistances) / temperature) > random.random():
                 tour = copy.copy(newTour)

        print(map(lambda x: "{} {}".format(x, tags[x]), tour))
        return list(map(lambda x: tags[x], tour))



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
        node = FieldTrip()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

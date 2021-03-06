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
from tagstore.srv import TagstoreAllTags, TagstoreAddTag
from plan_path.msg import PlanGoalAction, PlanGoalGoal
from transformations_odom.msg import PoseTF, PoseInMap
from plodding.srv import RotateQuarterPi



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

class Explorer:

    def __init__(self):
        rospy.init_node("explorer", log_level=rospy.DEBUG, anonymous=True)
        self.hasCamera = False
        self.isDone = False
        self.stateUnknownApproach = State_UnknownApproach()
        self.stateUnseenApproach = State_UnseenApproach()
        self.stateTagApproach = State_TagApproach()
        self.state = self.stateUnknownApproach
        self.rate = rospy.Rate(10)
        self.costmap = [[]]
        self.costmapInfo = None
        self.map = [[]]
        self.mapInfo = None
        self.cameraCostmap = [[]]
        self.cameraCostmapInfo = None
        self.poseTF = None
        self.assumedTagPose = PoseInMap(0, 0, 0)
        ### SUBSCRIBER
        rospy.logdebug("Explorer: Create Subscribers")
        rospy.logdebug("Explorer > Subscribers > '/camera/tag/visible'")
        rospy.Subscriber(
            name = '/camera/tag/visible',
            data_class=PoseInMap,
            callback=self._cb_tag_in_camera_pov
        )
        rospy.logdebug("Explorer > Subscribers > '/camera/tag/costmap'")
        rospy.Subscriber(
            name = "/camera/tag/costmap",
            data_class = OccupancyGrid,
            callback= self._cb_camera_pov_costmap
        )
        rospy.logdebug("Explorer > Subscribers > '/costmap_node/costmap/costmap'")
        rospy.Subscriber(
            name = "/costmap_node/costmap/costmap",
            data_class = OccupancyGrid,
            callback= self._cb_costmap
        )
        rospy.wait_for_message("/costmap_node/costmap/costmap", topic_type=OccupancyGrid)
        rospy.logdebug("Explorer > Subscribers > '/map'")
        rospy.Subscriber(
            name = "/map",
            data_class = OccupancyGrid,
            callback= self._cb_map
        )
        rospy.wait_for_message("/map", topic_type=OccupancyGrid)
        rospy.logdebug("Explorer > Subscribers > '/pose_tf'")
        rospy.Subscriber(
            name = "/pose_tf",
            data_class = PoseTF,
            callback= self._cb_pose_tf
        )
        rospy.wait_for_message("/pose_tf", topic_type=PoseTF)
        rospy.logdebug("Explorer: Get Services")
        ### Service
        rospy.logdebug("Explorer > Service > Await 'tagstore_tag_at'")
        rospy.wait_for_service(service="tagstore_tag_at")
        self.tagLookupSrv = rospy.ServiceProxy(
            name = "tagstore_tag_at",
            service_class=TagstoreAddTag
        )
        rospy.logdebug("Explorer > Service > Proxy for 'tagstore_tag_at'")

        rospy.logdebug("Explorer > Service > Await 'tagstore_addtag'")
        rospy.wait_for_service(service="tagstore_addtag")
        self.tagAddSrv = rospy.ServiceProxy(
            name = "tagstore_addtag",
            service_class=TagstoreAddTag
        )
        rospy.logdebug("Explorer > Service > Proxy for 'tagstore_tag_at'")

        rospy.logdebug("Explorer > Service > Await 'plodding_rotate_quarter_pi'")
        rospy.wait_for_service(service="plodding_rotate_quarter_pi")
        self.rotateQuarter = rospy.ServiceProxy(
            name = "plodding_rotate_quarter_pi",
            service_class=RotateQuarterPi
        )
        rospy.logdebug("Explorer > Service > Proxy for 'plodding_rotate_quarter_pi'")

        rospy.logdebug("Explorer: Await Action Client 'plan_path_action_server'")
        self.client = actionlib.SimpleActionClient("plan_path_action_server", PlanGoalAction)
        self.client.wait_for_server()
        rospy.logdebug("Explorer: Finished Action Client")

    def start(self):
        rospy.logdebug("Explorer: Start of Loop")
        self.state.enter(explorer = self)
        while not rospy.is_shutdown() and not self.isDone:
            self._changeState(self.state.run(explorer = self))
            if not self.hasCamera and self.stateUnseenApproach:
                # noinspection PyTypeChecker
                self._changeState(nextState = self.stateUnknownApproach)
            rospy.logdebug_throttle(1, "Explorer > State > {}".format(self.state.name()))
            self.rate.sleep()

        rospy.logdebug("Explorer: Finished Scenario")

    def _changeState(self, nextState):
        # type: (StateInterface) -> None
        if nextState != self.state:
            rospy.loginfo("Explorer: Transition {} State to {} State".format(self.state.name(), nextState.name()))
            self.state.cancel(explorer = self)
            self.state = nextState
            self.state.enter(explorer = self)

    def _cb_tag_in_camera_pov(self, data):
        # type: (PoseInMap) -> None
        # Check if there is a registered tag in the area
        # TODO Use actual coords given from data, check if x == y and y == x
        isExistingTag = self.tagLookupSrv(x = data.x, y =data.y).successful
        if not isExistingTag:
            self.assumedTagPose = data
            # Add tag to tagstore
            self.tagAddSrv(x = data.x, y =data.y)
            # noinspection PyTypeChecker
            self._changeState(nextState = self.stateTagApproach)

    def _cb_costmap(self, msg):
        self.costmap = self._reshapeOccupancyGrid(msg)
        self.costmapInfo = msg.info

    def _cb_map(self, msg):
        self.map = self._reshapeOccupancyGrid(msg)
        self.mapInfo = msg.info

    def _cb_camera_pov_costmap(self, msg):
        self.cameraCostmap = self._reshapeOccupancyGrid(msg)
        self.cameraCostmapInfo = msg.info

    def _cb_pose_tf(self, msg):
        self.poseTF = msg

    def sendPointToPlanPath(self, point, done_cb):
        goalData = PlanGoalGoal(x=point[0], y=point[1])
        rospy.logdebug("Explorer: Send Nav Target {}".format(point))
        self.client.send_goal(
            goal=goalData,
            done_cb=done_cb
        )

    @staticmethod
    def _reshapeOccupancyGrid(msg):
        return np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

    @staticmethod
    def extractValueAtOccupancyGrid(occupancyGrid, info, x, y):
        if -1 < x < info.width and -1 < y < info.height:
            # data comes in row-major order http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
            # first index is the row, second index the column
            return occupancyGrid[y][x]
        else:
            raise IndexError(
                "Coordinates out of gridmap, x: {}, y: {} must be in between: [0, {}], [0, {}]".format(
                    x, y, info.height, info.width))


class StateInterface:
    def __init__(self):
        pass

    def name(self):
        pass

    def enter(self, explorer):
        pass

    def run(self, explorer):
        # type: (Explorer) -> object
        return self

    def cancel(self, explorer):
        pass

class State_TagApproach(StateInterface):
    def __init__(self):
        StateInterface.__init__(self)
        self.busyEnter = False
        self.isActive = True

    def name(self): return "TagApproach"

    def enter(self, explorer):
        self.busyEnter = True
        self.isActive = True
        (x, y, angle) = explorer.assumedTagPose
        explorer.sendPointToPlanPath((x, y))
        self.busyEnter = False

    def _onDone(self, goalId, result):
        self.isActive = False

    def run(self, explorer):
        if not self.isActive:
            return explorer.stateUnknownApproach

        return self

    def cancel(self, explorer):
        explorer.client.cancel_all_goals()


class State_UnknownApproach(StateInterface):
    def __init__(self):
        StateInterface.__init__(self)
        self.busyEnter = False
        self.neighbourThreshold = 20
        self.isActive = True
        self.visited = []
        self.rotate = False
        self.rotateCount = 0

    def name(self): return "FindUnknown"

    def enter(self, explorer):
        self.busyEnter = True
        self.isActive = True
        point = self.findClosestUnknownOfSize(
            mapPose = explorer.poseTF.mapPose,
            costmap = explorer.costmap,
            map = explorer.map,
            info = explorer.costmapInfo,
            explorer=explorer,
            searchValue=-1,
            minCount=10
        )
        if not point is None:
            explorer.sendPointToPlanPath(
                point = point,
                done_cb=self._onDone
            )
        self.busyEnter = False

    def _onDone(self, goalId, result):
        self.rotate = True

    def run(self, explorer):

        if not self.isActive:
            if explorer.hasCamera:
                return explorer.stateUnseenApproach
            else:
                self.enter(explorer)
                return self
        elif self.rotate:
            if self.rotateCount < 4:
                self.rotateCount+=1
                explorer.rotateQuarter()
            else:
                self.rotate = False
                self.isActive = False

        return self

    def cancel(self, explorer):
        explorer.client.cancel_all_goals()

    def findClosestUnknownOfSize(self, mapPose, costmap, map, info, explorer, searchValue, minCount):
        expandList = [(mapPose.x, mapPose.y)]
        while len(expandList) > 0:
            curr = expandList.pop(0)
            self.visited.append(curr)
            mapVal = min(0, explorer.extractValueAtOccupancyGrid(
                occupancyGrid=map,
                info=info,
                x=curr[0],
                y=curr[1]
            ))
            currValue = explorer.extractValueAtOccupancyGrid(
                occupancyGrid = costmap,
                info = info,
                x = curr[0],
                y = curr[1]
            ) + mapVal
            # rospy.logdebug("Look at: {} with cost of {}, len {}".format(curr, currValue, len(expandList)))
            neighbours = self.neighborsForCell(
                costmap=costmap,
                cell=curr,
                visited=self.visited,
                expandList=expandList
            )
            if currValue == searchValue:
                blobSizeCount = 1
                localVisited = self.visited[:]
                localExpand = neighbours[:]
                localPoint = None
                # rospy.logdebug("Inner Look at: {} with cost of {}".format(curr, currValue))
                while len(localExpand) > 0 and blobSizeCount < minCount:
                    localPoint = localExpand.pop(0)
                    mapVal = min(0, explorer.extractValueAtOccupancyGrid(
                        occupancyGrid=map,
                        info=info,
                        x=curr[0],
                        y=curr[1]
                    ))
                    localValue = explorer.extractValueAtOccupancyGrid(
                        occupancyGrid = costmap,
                        info = info,
                        x = localPoint[0],
                        y = localPoint[1]
                    ) + mapVal
                    localNeighbours = self.neighborsForCell(
                        costmap=costmap,
                        cell=localPoint,
                        visited=localVisited,
                        expandList=localExpand
                    )
                    if localValue == searchValue:
                        blobSizeCount += 1

                    localExpand.extend(localNeighbours)

                if blobSizeCount < minCount:
                    # Blob in full size was too small all visited nodes can be disregarded
                    self.visited = localVisited
                else:
                    return localPoint
            else:
                expandList.extend(neighbours)
        return None


    def neighborsForCell(self, costmap, cell, visited, expandList):
        l = []
        for direction in cardinals:
            target = add(cell, direction)
            if not target in visited and not target in expandList:
                try:
                    if costmap[target[0]][target[1]] < self.neighbourThreshold:
                        l.append(target)
                except IndexError:
                    pass
                    # rospy.logwarn("PlanPath: Costmap index access outside of bounds: {}".format(target))
        return l

class State_UnseenApproach(StateInterface):
    def __init__(self):
        StateInterface.__init__(self)
        self.busyEnter = False
        self.neighbourThreshold = 20
        self.isActive = False
        self.visited = []
        self.rotate = False
        self.rotateCount = 0

    def name(self): return "FindUnseen"

    def enter(self, explorer):
        if explorer.cameraCostmap is None or explorer.cameraCostmapInfo is None:
            return
        self.busyEnter = True
        self.isActive = True
        point = self.findClosestUnseenOfSize(
            mapPose = explorer.poseTF.mapPose,
            cameramap= explorer.cameraCostmap,
            costmap= explorer.costmap,
            info = explorer.cameraCostmapInfo,
            explorer=explorer,
            searchValue=-1,
            minCount=10
        )
        if not point is None:
            explorer.sendPointToPlanPath(
                point = point,
                done_cb=self._onDone
            )
        self.busyEnter = False

    def _onDone(self, goalId, result):
        self.rotate = True

    def run(self, explorer):

        if not self.isActive:
            return explorer.stateUnknownApproach
        elif self.rotate:
            if self.rotateCount < 4:
                self.rotateCount+=1
                explorer.rotateQuarter()
            else:
                self.isActive = False

        return self

    def cancel(self, explorer):
        explorer.client.cancel_all_goals()

    def findClosestUnseenOfSize(self, mapPose, cameramap, costmap, info, explorer, searchValue, minCount):
        expandList = [(mapPose.x, mapPose.y)]
        while len(expandList) > 0:
            curr = expandList.pop(0)
            self.visited.append(curr)
            currValue = explorer.extractValueAtOccupancyGrid(
                occupancyGrid = cameramap,
                info = info,
                x = curr[0],
                y = curr[1]
            )
            rospy.logdebug("Look at: {} with cost of {}, len {}".format(curr, currValue, len(expandList)))
            neighbours = self.neighborsForCell(
                costmap=cameramap,
                cell=curr,
                visited=self.visited,
                expandList=expandList
            )
            if currValue == searchValue:
                mapVal = explorer.extractValueAtOccupancyGrid(
                    occupancyGrid=costmap,
                    info=info,
                    x=curr[0],
                    y=curr[1]
                )
                if mapVal < self.neighbourThreshold:

                    blobSizeCount = 1
                    localVisited = self.visited[:]
                    localExpand = neighbours[:]
                    localPoint = None
                    rospy.logdebug("Inner Look at: {} with cost of {}".format(curr, currValue))
                    while len(localExpand) > 0 and blobSizeCount < minCount:
                        localPoint = localExpand.pop(0)
                        mapVal = min(0, explorer.extractValueAtOccupancyGrid(
                            occupancyGrid=costmap,
                            info=info,
                            x=curr[0],
                            y=curr[1]
                        ))
                        localValue = explorer.extractValueAtOccupancyGrid(
                            occupancyGrid = cameramap,
                            info = info,
                            x = localPoint[0],
                            y = localPoint[1]
                        ) + mapVal
                        localNeighbours = self.neighborsForCell(
                            costmap=cameramap,
                            cell=localPoint,
                            visited=localVisited,
                            expandList=localExpand
                        )
                        if localValue == searchValue:
                            blobSizeCount += 1

                        localExpand.extend(localNeighbours)

                    if blobSizeCount < minCount:
                        # Blob in full size was too small all visited nodes can be disregarded
                        self.visited = localVisited
                    else:
                        return localPoint
            else:
                expandList.extend(neighbours)
        return None


    def neighborsForCell(self, costmap, cell, visited, expandList):
        l = []
        for direction in cardinals:
            target = add(cell, direction)
            if not target in visited and not target in expandList:
                try:
                    if costmap[target[0]][target[1]] < self.neighbourThreshold:
                        l.append(target)
                except IndexError:
                    pass
                    # rospy.logwarn("PlanPath: Costmap index access outside of bounds: {}".format(target))
        return l

def main():
    try:
        node = Explorer()
        node.start()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

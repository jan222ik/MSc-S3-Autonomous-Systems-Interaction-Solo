#! /usr/bin/env python

import os
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Bool
from tagstore.srv import TagstoreAddTag, TagstoreAddTagResponse

class Tagstore:

    def __init__(self):
        rospy.init_node('tagstore', log_level=rospy.DEBUG, anonymous=True)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo("Tagstore: Startup")
        self.tagsPath = rospy.get_param("~tagsFilepath", default = "/home/jan/catkin_ws/exec/tags.csv")
        self.tagsList = []

        self.rvisPointPublisher = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=100)
        self.mapInfo = MapMetaData()
        self.mapInfo = rospy.wait_for_message("map", OccupancyGrid).info
        self.rvisMarkerArray = MarkerArray()

        self.srvAddTag = rospy.Service("tagstore-addtag", TagstoreAddTag, self._srvAddTag)

        self._loadTagsFromFile()

        # self.tagsList.append(Tag(47, 11, 3))
        # self._saveTagsToFile()
        rospy.spin()


    def _shutdown(self):
        """
        Shutdown Callback.
        """
        rospy.loginfo("Tagstore: Shutdown")
        self._saveTagsToFile()


    def _loadTagsFromFile(self):
        """
        Loads tags from csv.
        """
        rospy.loginfo("Tagstore: Load Tags")
        try:
            with open(self.tagsPath, "r") as f:
                lines = f.readlines()
                for idx, csvRow in enumerate(lines):
                    rospy.logdebug("Tagstore: Import {} with CSV Row: {}".format(idx, csvRow))
                    cells = csvRow.strip().split(',')
                    tag = Tag(int(cells[0]), int(cells[1]), int(cells[2]))
                    rospy.loginfo("X:{} Y:{} Global-ID:{}".format(*cells))
                    self.tagsList.append(tag)
                    self._publishRvisPoints(tag)
            rospy.loginfo("Tagstore: Loaded Tags")
        except IOError:
            rospy.logerr("Tagstore: File for tags not found.")

    def _saveTagsToFile(self):
        """
            Writes tags to csv.
        """
        rospy.loginfo("Tagstore: Saving Tags")
        try:
            with open(self.tagsPath, "w") as f:
                for tag in self.tagsList:
                    f.write(tag.asCSV() + "\n")
            rospy.loginfo("Tagstore: Saved {} Tags".format(len(self.tagsList)))
        except IOError:
            rospy.logerr("Tagstore: File for tags not found.")

    def _srvAddTag(self, srvData):
        x = srvData.x
        y = srvData.y

        findExisting = self._findTagAt(x, y)
        res = Bool()
        res.data = False

        if findExisting is None:
            tag = Tag(x, y, len(self.tagsList) + 1)
            self.tagsList.append(tag)
            self._publishRvisPoints(tag)
            res.data = True

        rospy.logdebug("Tagstore > Service > AddTag: Res success?: {}".format(res.data))
        return TagstoreAddTagResponse(res)

    def _findTagAt(self, x, y):
        tagDetectRadius = 10 * self.mapInfo.resolution
        for tag in self.tagsList:
            if tag.checkXY(x, y, tagDetectRadius):
                return tag
        return None


    def _publishRvisPoints(self, tag):
        """
        Publish Tag Position for RVIS.
        """
        rospy.loginfo("Tagstore: Publish Tag to RVis: {}".format(tag.asCSV()))
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "rvismarker"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = tag.x * self.mapInfo.resolution + self.mapInfo.origin.position.x
        marker.pose.position.y = tag.y * self.mapInfo.resolution + self.mapInfo.origin.position.y
        marker.pose.position.z = 1


        self.rvisMarkerArray.markers.append(marker)
        idx = 0
        for m in self.rvisMarkerArray.markers:
            m.id = idx
            idx+=1

        self.rvisPointPublisher.publish(self.rvisMarkerArray)

        rospy.sleep(0.01)







class Tag:
    def __init__(self, x, y, globalId):
        self.x = x
        self.y = y
        self.globalId = globalId

    def asCSV(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.globalId)

    def checkX(self, x, detectRadius):
        lower = self.x - detectRadius
        upper = self.x + detectRadius
        return lower <= x <= upper

    def checkY(self, y, detectRadius):
        lower = self.y - detectRadius
        upper = self.y + detectRadius
        return lower <= y <= upper

    def checkXY(self, x, y, detectRadius):
        return self.checkX(x, detectRadius) and self.checkY(y, detectRadius)





def main():
    try:
        node = Tagstore()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()


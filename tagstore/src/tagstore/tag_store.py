#! /usr/bin/env python

import os
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, MapMetaData

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

    def _publishRvisPoints(self, tag):
        """
        Publish Tag Position for RVIS.
        """
        rospy.loginfo("Tagstore: Publish Tag to RVis: {}".format(tag.asCSV()))
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 10
        marker.color.r = 1.0
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = (tag.x * self.mapInfo.resolution) + self.mapInfo.origin.position.x
        marker.pose.position.y = (tag.y * self.mapInfo.resolution) + self.mapInfo.origin.position.y
        marker.pose.position.z = 1
        marker.id = tag.globalId

        self.rvisMarkerArray.markers.append(marker)
        self.rvisPointPublisher.publish(self.rvisMarkerArray)

        rospy.sleep(0.01)







class Tag:
    def __init__(self, x, y, globalId):
        self.x = x
        self.y = y
        self.globalId = globalId

    def asCSV(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.globalId)





def main():
    try:
        node = Tagstore()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()


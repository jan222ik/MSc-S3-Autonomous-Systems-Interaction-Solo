#!/usr/bin/env python

from __future__ import print_function

import rospy
import sys
from std_msgs.msg import Bool
from subprocess import call
from localization.srv import MapSaveLoad, MapSaveLoadResponse


class MapHandler:
    def __init__(self):
        self.base_call_save = 'rosrun map_server map_saver -f ~/catkin_ws/src/MSc-S3-Autonomous-Systems-Interaction-Solo/localization/maps/'
        self.base_call_load = 'rosrun map_server map_server ~/catkin_ws/src/MSc-S3-Autonomous-Systems-Interaction-Solo/localization/maps/'
        self.srvSaveLoad = rospy.Service("/map_save_load", MapSaveLoad, self._save_load_map)

    def _save_load_map(self, map_srv):
        rospy.loginfo("MAP HANDLING CALL")
        call_string = ""
        if map_srv.mode == 0:
            call_string = self.base_call_save + map_srv.map_name
        else:
            call_string = call_string = self.base_call_load + map_srv.map_name
        rospy.print("CALL STRING: ", call_string)
        call(call_string)
        res = Bool()
        res.data = True
        return MapSaveLoadResponse(res)


def main(args):
    mh = MapHandler()
    rospy.init_node('map_handler', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)

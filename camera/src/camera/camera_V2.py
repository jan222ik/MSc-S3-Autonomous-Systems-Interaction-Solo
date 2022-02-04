#!/usr/bin/env python
from __future__ import print_function

import math

import sys
import rospy
import cv2
import tf
from sensor_msgs.msg import Image, CameraInfo, CompressedImage, PointCloud2
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from image_geometry.cameramodels import PinholeCameraModel
from tagstore.srv import TagstoreAddTag, TagstoreResetRVis
from transformations_odom.msg import PoseInMap
import numpy as np
from subprocess import call

class CameraV2:

    def __init__(self):
        rospy.init_node("camera_v2", anonymous=True)
        self.bridge = CvBridge()
        self.raspi_cam_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.raspi_callback)
        self.img_raw_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.img_raw_callback)
        self.img_raw_depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.depth_pointcloud_sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.point_cloud_callback)

    def raspi_callback(self, data):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        self.do_detection(cv_image)

    def img_raw_callback(self, data):
        print("IMAGE DETECTED")
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.do_detection(cv_image)

    def depth_callback(self, data):
        pass

    def point_cloud_callback(self, data):
        print("POINT CLOUD DETECTED")
        row_step = data.row_step
        height = data.height
        # mat = np.reshape(np.array(data.data), (row_step, height))

    def do_detection(self, cv_image):
        cv2.imshow("test", cv_image)



def main(args):
    td = CameraV2()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)


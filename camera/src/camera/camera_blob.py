#!/usr/bin/env python
from __future__ import print_function

import math

import sys
import rospy
import cv2
import tf
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from image_geometry.cameramodels import PinholeCameraModel
from tagstore.srv import TagstoreAddTag, TagstoreResetRVis
from transformations_odom.msg import PoseInMap
import numpy as np
from subprocess import call


class BlobDetector:

    def __init__(self):
        rospy.init_node('blob_detector', anonymous=True)
        self.mapInfo = MapMetaData()
        # self.mapInfo = rospy.wait_for_message("map", OccupancyGrid).info
        self.is_calculating = False
        self.image_pub = rospy.Publisher("/blob/image_topic_tag", Image)
        # rospy.on_shutdown(self._shutdown)
        self.bridge = CvBridge()
        self.raspi_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.raspi_callback)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.img_raw_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.camera_rgb_info = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.cam_callback)
        self.cam = PinholeCameraModel()
        self.camera_info = None
        self.not_cam_setup = True
        self.depth_image = None
        self.pose = None
        self.saved_map = False
        self.cur_pose = None
        print("Setup done")

    def map_pose_callback(self, data):
        self.pose = (data.x, data.y)

    def add_tag_with_tagstore(self, x, y):
        rospy.wait_for_service("tagstore-addtag")
        try:
            tag_add = rospy.ServiceProxy("tagstore-addtag", TagstoreAddTag)
            resp = tag_add(x, y)
            print("Adding tag: ", resp)
        except rospy.ServiceException as e:
            print("Service call to add tag failed: %s" % e)


    def depth_callback(self, data):
        try:
            NewImg = self.bridge.imgmsg_to_cv2(data, "passthrough")
            self.depth_image = NewImg
            # cv2.imshow("depth.png", NewImg)
        except CvBridgeError as e:
            print(e)

    def cam_callback(self, data):
        if self.not_cam_setup:
            self.camera_info = data
            self.cam.fromCameraInfo(data)
            self.not_cam_setup = False

    def convert_point(self, x, y, base_frame, target_frame):
        listener = tf.TransformListener()
        listener.waitForTransform(base_frame, target_frame, rospy.Time(0), rospy.Duration(4.0))
        stamp_point = PointStamped()
        stamp_point.header.frame_id = base_frame
        stamp_point.header.stamp = rospy.Time(0)
        stamp_point.point.x = x
        stamp_point.point.y = y
        stamp_point.point.z = 0.0
        return listener.transformPoint(target_frame, stamp_point)

    def raspi_callback(self, data):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        self.do_detection(cv_image)

    def img_raw_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.do_detection(cv_image)

    def do_detection(self, cv_image):
        # Set up the detector with default parameters.
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 200

        params.filterByColor = True
        params.blobColor = 150

        # Filter by Area.
        params.filterByArea = False
        params.minArea = 1500

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.87

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3:
            detector = cv2.SimpleBlobDetector(params)
        else:
            detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs.
        keypoints = detector.detect(cv_image)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0, 0, 255),
                                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show keypoints
        cv2.imshow("Keypoints", im_with_keypoints)
        cv2.waitKey(0)


def main(args):
    td = BlobDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

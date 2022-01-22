#!/usr/bin/env python

from __future__ import print_function

import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
import math
from std_srvs.srv import Empty


class Localizer:
    def __init__(self):
        self.pose_estimate = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_estimate_callback)
        # self.occupancy_grid = rospy.wait_for_message("/map", OccupancyGrid)
        self.epsilon = 0.5
        # --- Service ---
        rospy.wait_for_service('global_localization')
        self.global_localisation = rospy.ServiceProxy('global_localization', Empty)

    def pose_estimate_callback(self, estimated_pose):
        # Deconstructing PoseWithCovarianceStamped
        ## Header
        header = estimated_pose.header
        seq = header.seq
        time_stamp = header.stamp
        frame_id = header.frame_id

        ## Pose with covariance
        pose_with_cov = estimated_pose.pose

        ### Pose
        pose = pose_with_cov.pose

        #### Point
        point = pose.position
        point_x = point.x
        point_y = point.y
        point_z = point.z

        #### Quaternion
        quaternion = pose.orientation
        quat_x = quaternion.x
        quat_y = quaternion.y
        quat_z = quaternion.z
        quat_w = quaternion.w

        ### Covariance (float64[36])
        covariance = pose_with_cov.covariance
        cov_arr = np.asarray(covariance)
        cov_mat = np.reshape(cov_arr, (6, 6))

        self.global_localisation()

        if self._calc_cov_area(cov_mat) < self.epsilon:
            print("LOCALIZED!!!!")

    def _calc_cov_area(self, cov_mat):
        """
        Calculate area of covariance ellipsis
        """
        # Cut off unneeded parts (since we cannot fly with the turtlebot, they are 0 anyway)
        cov_reduced = cov_mat[0:2, 0:2]

        # calculate eigenvectors/eigenvalues to get a and b
        eig_val, eig_vec = np.linalg.eig(np.linalg.inv(cov_reduced))
        eigen_x, eigen_y = eig_vec[:, 0]
        a, b = 2 / np.sqrt(eig_val)
        # area of ellipse is a * b * pi
        area = a * b * math.pi
        print("AREA OF ELLIPSE: ", str(area))
        return area

def main(args):
    loc = Localizer()
    rospy.init_node('localize_robot', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)

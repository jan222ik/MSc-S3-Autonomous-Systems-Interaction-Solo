#!/usr/bin/env python

from __future__ import print_function

import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
import math
from std_srvs.srv import Empty
import actionlib
from localization.msg import LocalizationState, LocalizationAction, LocalizationResult, LocalizationFeedback


class Localizer:
    def __init__(self):
        # Subscribe to amcl to get localization PoseWithCovariance
        self.pose_estimate = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._pose_estimate_callback)

        # get service for global localization
        rospy.wait_for_service('global_localization')
        self.global_localisation = rospy.ServiceProxy('global_localization', Empty)

        # Set up action server (result, feedback, server) and start it
        self.loc_feedback = LocalizationFeedback()
        self.loc_result = LocalizationResult()
        self.loc_action = actionlib.SimpleActionServer(name='localize_me',
                                                       ActionSpec=LocalizationAction,
                                                       execute_cb=self._locate_me,
                                                       auto_start=False)

    def _pose_estimate_callback(self, estimated_pose):
        # Deconstructing PoseWithCovarianceStamped
        ## Header
        header = estimated_pose.header
        seq = header.seq
        time_stamp = header.stamp
        frame_id = header.frame_id

        ## Pose with covariance
        pose_with_cov = estimated_pose.pose
        self.loc_feedback.cur_loc_state.pose_w_cov = pose_with_cov

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
        self.loc_feedback.cur_loc_state.ellipse_area = self._calc_cov_area(cov_mat)

    def _locate_me(self, goal):
        # perform global localization once at the beginning
        # note: for small maps with few/no obstacles, this is actually counterproductive!
        self.global_localisation()
        # helper variables
        r = rospy.Rate(1)
        success = True
        # Initialize locate message
        self.loc_feedback.cur_loc_state.epsilon = goal.epsilon
        self.loc_feedback.cur_loc_state.ellipse_area = np.Inf
        while self.loc_feedback.cur_loc_state.ellipse_area > goal.epsilon:
            # TODO: move around
            if self.loc_action.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % 'localize_me')
                self.loc_action.set_preempted()
                success = False
                break
            self.loc_action.publish_feedback(self.loc_feedback)
        if success:
            self.loc_result.loc_end_state = self.loc_feedback.cur_loc_state
            rospy.loginfo('%s: Succeeded' % 'localize_me')
            self.loc_action.set_succeeded(self.loc_result)

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
    loc.loc_action.start()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)

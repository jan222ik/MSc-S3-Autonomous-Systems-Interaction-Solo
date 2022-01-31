#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import numpy as np

import math


class WallFollowerTwo:

    def __init__(self):
        rospy.init_node('WallFollower')
        self.rate = rospy.Rate(20)
        self.state = 0
        self.botVelPub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laserSub = rospy.Subscriber('/scan', LaserScan, self.updateLaserData)
        self.laserData = None

    def updateLaserData(self, msg):
        self.laserData = msg.ranges

    def step(self):
        twist = self.sensePlan()
        self.botVelPub.publish(twist)

        self.rate.sleep()

    def sensePlan(self):
        if not self.laserData:
            print("Laser data not present")
            return Twist()

        data = np.array(self.laserData)
        k = 5
        closestK = np.argpartition(data, k)
        print(closestK[:k])
        #  print(data[closestK[:k]])

        d = 0.5  # Distance to keep away from wall
        twist = Twist()

        if data[0] < d:
            # Frontal obstacle detected -> avoidance
            print("Evade front - Turn Left")
            twist.angular.z = 0.2
        elif 260 < closestK[0] < 280:
            # The closest measurement is to the right side
            print("Follow Wall")
            twist.linear.x = 0.2
            # Correct steering
            steer = max(min(closestK[0] - 270, 1), -1)
            if data[closestK[0]] < (d / 1.5):
                # Override steer if entered distance threshold
                steer = 1
            twist.angular.z = 0.005 * steer
        elif not (120 < closestK[0] < 240) and data[closestK[0]] < d:
            # Obstacle ahead, need to turn
            # Fast rotate based on sensor provided angle
            if 270 < closestK[0] or closestK[0] < 90:
                print("Turn Left")
                twist.angular.z = 0.2
            else:
                print("Turn Right")
                twist.angular.z = -0.05
        else:
            # Dive to wall or clear back of robot if closest walls are behind
            if not (120 < closestK[0] < 240):
                print("Lost wall")
            else:
                print("Forward to clear back of robot")
            twist.linear.x = 0.05
        return twist


def main():
    wall = WallFollowerTwo()
    try:
        while not rospy.is_shutdown():
            wall.step()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()

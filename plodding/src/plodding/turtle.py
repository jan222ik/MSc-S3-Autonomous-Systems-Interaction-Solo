#! /usr/bin/env python

import rospy
import actionlib
import math
#import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from transformations_odom.msg import PoseTF, PoseInMap
#from traverse_path.msg import TraversePathAction, TraversePathActionResult, TraversePathActionFeedback, TraversePathResult
from geometry_msgs.msg import Twist, Pose
from plodding.msg import PlodAction, PlodActionResult, PlodResult, PlodActionFeedback
from turtlebot3_msgs.msg import Sound

class PloddingTurtle:
    def __init__(self):
        rospy.init_node("plodding", log_level=rospy.DEBUG, anonymous=True)
        self.rate = rospy.Rate(10)
        self.goalDistanceThreshold = 0.05
        self.goalAngleThreshold = 0.2
        self.collisionThreshold = 1
        self.LINEAR_MAX_SPEED = 0.3
        self.state = State_Idle()
        rospy.loginfo("Plodding: Startup")
        self.pubTwist = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pubSound = rospy.Publisher("sound", Sound, queue_size=10)
        self.currentPose = Pose()
        self.mapPose = PoseInMap()
        self.goalPose = Pose()
        self.subRobotPose = rospy.Subscriber('pose_tf', PoseTF, self._subNextPose)
        rospy.logdebug("Plodding: Starting Action Server")
        self.actionServer = actionlib.SimpleActionServer("plodding_action_server", PlodAction, execute_cb=self._nextActionPose, auto_start = False)
        self.actionServer.start()
        rospy.logdebug("Plodding: Started Action Server")

    def _subNextPose(self, data):
        self.mapPose = data.mapPose
        self.currentPose = data.originalPose


    def _nextActionPose(self, data):
        rospy.logdebug("Plodding: Next Action Pose {}".format(data))
        self.externalCancel = False
        self.goalPose = data.target

        result = PlodActionResult()
        result.result = PlodResult()
        result.result.plod_has_finished = Bool()
        feedback = PlodActionFeedback()

        while not rospy.is_shutdown() and not self.externalCancel:
            if self.actionServer.is_preempt_requested():
                rospy.loginfo("Plodding: Requested Cancel")
                self.stop()
                self._changeState(State_Idle())
                self.externalCancel = True
            else:
                self._changeState(nextState = self.state.run(self))


            feedback.feedback.state = String(data=self.state.name())
            self.actionServer.publish_feedback(feedback.feedback)

        result.result.plod_has_finished.data = False
        self.actionServer.set_succeeded(result.result)

    def _changeState(self, nextState):
        if nextState == self.state:
            rospy.loginfo("Plodding: Transition {} State to {} State".format(self.state.name(), nextState.name()))
            self.state = nextState

    def playSound(self):
        self.pubSound.publish(Sound(value=4))

    @staticmethod
    def calcDistance(first, second, roundingPos=4):
        """
            Calculate euclidean distance between two poses.
        """
        return math.sqrt(
            pow((first.position.x - round(second.position.x, roundingPos)), 2)
            + pow((first.position.y - round(second.position.y, roundingPos)), 2)
        )

    @staticmethod
    def calcAngle(first, second, roundingPos=4):
        y = first.position.y - round(second.position.y, roundingPos)
        x = first.position.x - round(second.position.x, roundingPos)
        angle = math.atan2(y, x)
        return angle

    @staticmethod
    def angularSpeedFor(currentAngle, targetAngle):
        speed = 0.18
        radCurrent = currentAngle.assurePositiveAndIn2Pi()
        radTarget = targetAngle.assurePositiveAndIn2Pi()
        diff = (radCurrent - radTarget) % (2 * math.pi)
        if not diff < math.pi:
            speed*=-1

        return speed


    @staticmethod
    def assurePositiveAndIn2Pi(rad):
        return (rad + 2 * math.pi) % (2 * math.pi)

    def publishTwistMessage(self, linearSpeed, angularSpeed):
        twist = Twist()
        twist.linear.x = linearSpeed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = angularSpeed
        self.pubTwist.publish(twist)
        self.rate.sleep()

    def stop(self):
        self.publishTwistMessage(0,0)

    def isInGoalRange(self):
        return self.calcDistance(
            first=self.goalPose,
            second=self.currentPose
        ) <= self.goalDistanceThreshold

    def isInGoalRotationRange(self):
        return self.calcAngle(
            first=self.goalPose,
            second=self.currentPose
        ) <= self.goalAngleThreshold

class StateInterface:
    def name(self):
        pass
    def run(self, turtle):
        pass



class State_Idle(StateInterface):
    def name(self): return "Idle"

    def run(self, turtle):
        return self

class State_Plodding(StateInterface):
    def name(self): return "Plodding"

    def run(self, turtle):
        isInGoalDistanceRange = turtle.isInGoalDistanceRange()
        isInGoalRotationRange = turtle.isInGoalRotationRange()
        if isInGoalDistanceRange and isInGoalRotationRange:
            # Exit Goal Reached
            turtle.stop()
            return State_Timeout() # TODO pass idle or timeout state based on goal message param

        linear = 0
        angular = 0

        if not isInGoalDistanceRange:
            linear = turtle.LINEAR_MAX_SPEED

        if not isInGoalRotationRange:
            # Location Reached but Rotation is still needed
            angular = turtle.angularSpeedFor(
                currentAngle=turtle.self.calcAngle(
                    first=turtle.goalPose,
                    second=turtle.currentPose
                ),
                targetAngle=turtle.mapPose.angle
            )

        turtle.publishTwistMessage(
            linearSpeed=linear,
            angularSpeed=angular
        )

        return self



class State_Timeout(StateInterface):

    def __init__(self):
        self.time = None
        self.timeoutConstant = 2000

    def name(self): return "Timeout"

    def run(self, turtle):
        if self.time is None: # First Run: Start Timer
            self.time = rospy.get_rostime().secs + self.timeoutConstant
            turtle.playSound()

        if self.time <= rospy.get_rostime().secs:
            self.time = None
            return State_Idle()



        return self

class State_AvoidCollision:
    def name(self): return "Avoid Collision"

    def run(self, turtle):
        return self


def main():
    try:
        node = PloddingTurtle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

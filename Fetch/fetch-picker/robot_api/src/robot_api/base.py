#! /usr/bin/env python

from geometry_msgs.msg import Twist
import rospy

TwistMsg = Twist

class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = robot_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        twist_msg = TwistMsg()
        # TODO: Fill out msg
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed
        # TODO: Publish msg
        self.pub.publish(twist_msg)
        # rospy.logerr('Not implemented.')

    def stop(self):
        """Stops the mobile base from moving.
        """
        twist_msg = TwistMsg()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self.pub.publish(twist_msg)
        # TODO: Publish 0 velocity
        # rospy.logerr('Not implemented.')

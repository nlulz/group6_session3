#!/usr/bin/env python3

# Import Dependencies
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import time


class TurtlesimStraightsAndTurns:

    def __init__(self):

        # Initialize class variables
        self.last_distance = 0
        self.goal_distance = 0
        self.dist_goal_active = False
        self.forward_movement = True

        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Initialize subscribers
        rospy.Subscriber("/turtle_dist", Float64, self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)

        # Initialize publishers
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Timer (main loop)
        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        rospy.loginfo("Initialized node!")

        rospy.spin()

    def pose_callback(self, msg):
        pass

    def distance_callback(self, msg):
        self.last_distance = msg.data

    def goal_angle_callback(self, msg):
        pass

    def goal_distance_callback(self, msg):

        # Set new goal distance
        self.goal_distance = msg.data

        # Activate movement
        self.dist_goal_active = True
        self.forward_movement = True

        # Reset distance tracker
        self.last_distance = 0

        rospy.loginfo("New goal distance received: %f", self.goal_distance)

    def timer_callback(self, msg):

        vel_msg = Twist()

        if self.dist_goal_active:

            # Check if goal reached
            if self.last_distance >= self.goal_distance:

                vel_msg.linear.x = 0
                vel_msg.angular.z = 0

                self.dist_goal_active = False

                rospy.loginfo("Goal distance reached")

            else:

                if self.forward_movement:
                    vel_msg.linear.x = 1.0
                    vel_msg.angular.z = 0

        # Publish velocity
        self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':

    try:
        turtlesim_straights_and_turns_class_instance = TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException:
        pass

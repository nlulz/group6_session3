#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose


class StraightsAndTurnsTurtle:
    def __init__(self):
        rospy.init_node('straights_and_turns_turtle', anonymous=True)

        self.cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        rospy.Subscriber('/goal_distance', Float64, self.goal_distance_callback)
        rospy.Subscriber('/goal_angle', Float64, self.goal_angle_callback)
        rospy.Subscriber('/goal_position', Point, self.goal_position_callback)

        self.pose = None

        self.active_goal_type = None

        self.goal_distance = 0.0
        self.goal_angle = 0.0
        self.goal_position = None

        self.start_x = None
        self.start_y = None
        self.start_theta = None

        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)

        rospy.loginfo("Node started")

    def pose_callback(self, msg):
        self.pose = msg

    # ✅ FIXED (no blocking)
    def goal_distance_callback(self, msg):
        self.goal_distance = msg.data
        self.active_goal_type = 'distance'
        self.start_x = None  # reset so it initializes later

        rospy.loginfo(f"Distance goal: {self.goal_distance}")

    # ✅ FIXED
    def goal_angle_callback(self, msg):
        self.goal_angle = msg.data
        self.active_goal_type = 'angle'
        self.start_theta = None

        rospy.loginfo(f"Angle goal: {self.goal_angle}")

    def goal_position_callback(self, msg):
        self.goal_position = msg
        self.active_goal_type = 'position'

        rospy.loginfo(f"Position goal: ({msg.x}, {msg.y})")

    def stop_turtle(self):
        self.cmd_pub.publish(Twist())

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def execute_distance_goal(self):
        if self.pose is None:
            return

        # Initialize start position only once
        if self.start_x is None:
            self.start_x = self.pose.x
            self.start_y = self.pose.y

        cmd = Twist()

        if self.goal_distance == 0:
            self.stop_turtle()
            self.active_goal_type = None
            return

        dx = self.pose.x - self.start_x
        dy = self.pose.y - self.start_y
        travelled = math.sqrt(dx**2 + dy**2)

        target = abs(self.goal_distance)

        if travelled < target:
            cmd.linear.x = 1.5 if self.goal_distance > 0 else -1.5
        else:
            self.stop_turtle()
            self.active_goal_type = None
            return

        self.cmd_pub.publish(cmd)

    def execute_angle_goal(self):
        if self.pose is None:
            return

        # Initialize start angle once
        if self.start_theta is None:
            self.start_theta = self.pose.theta

        cmd = Twist()

        if self.goal_angle == 0:
            self.stop_turtle()
            self.active_goal_type = None
            return

        turned = self.normalize_angle(self.pose.theta - self.start_theta)
        target = abs(self.goal_angle)

        if abs(turned) < target:
            cmd.angular.z = 1.0 if self.goal_angle > 0 else -1.0
        else:
            self.stop_turtle()
            self.active_goal_type = None
            return

        self.cmd_pub.publish(cmd)

    def execute_position_goal(self):
        if self.pose is None or self.goal_position is None:
            return

        cmd = Twist()

        dx = self.goal_position.x - self.pose.x
        dy = self.goal_position.y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.05:
            self.stop_turtle()
            self.active_goal_type = None
            return

        target_theta = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_theta - self.pose.theta)

        if abs(angle_error) > 0.05:
            cmd.angular.z = 1.0 if angle_error > 0 else -1.0
        else:
            cmd.linear.x = 1.5

        self.cmd_pub.publish(cmd)

    def control_loop(self, event):
        if self.active_goal_type is None:
            return

        if self.active_goal_type == 'distance':
            self.execute_distance_goal()
        elif self.active_goal_type == 'angle':
            self.execute_angle_goal()
        elif self.active_goal_type == 'position':
            self.execute_position_goal()


if __name__ == '__main__':
    try:
        StraightsAndTurnsTurtle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

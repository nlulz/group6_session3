#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, AprilTagDetectionArray


class TargetFollower:
    def __init__(self):
        rospy.init_node("target_follower_node", anonymous=True)

        # Robot name
        # Your Duckiebot name is mybota003087
        self.veh = rospy.get_param("~veh", "mybota003087")

        # Publisher: sends velocity commands to the Duckiebot
        self.cmd_pub = rospy.Publisher(
            f"/{self.veh}/car_cmd_switch_node/cmd",
            Twist2DStamped,
            queue_size=1
        )

        # Subscriber: receives AprilTag detections
        self.tag_sub = rospy.Subscriber(
            f"/{self.veh}/apriltag_detector_node/detections",
            AprilTagDetectionArray,
            self.tag_callback,
            queue_size=1
        )

        # -----------------------------
        # TUNING VALUES
        # -----------------------------

        # Behaviour 1: seek object when no tag is visible
        self.seek_omega = 0.9

        # Behaviour 2: look at object when tag is detected
        self.kp = 5.0
        self.dead_zone = 0.05
        self.min_omega = 0.45
        self.max_omega = 2.5

        # Small delay to slow command updates
        self.command_delay = 0.1

        rospy.on_shutdown(self.clean_shutdown)

        rospy.loginfo("======================================")
        rospy.loginfo("Target Follower Node Started")
        rospy.loginfo(f"Robot name: {self.veh}")
        rospy.loginfo(f"Publishing to: /{self.veh}/car_cmd_switch_node/cmd")
        rospy.loginfo(f"Subscribing to: /{self.veh}/apriltag_detector_node/detections")
        rospy.loginfo("Behaviour 1: Seek object when no AprilTag is visible")
        rospy.loginfo("Behaviour 2: Rotate to look at AprilTag when detected")
        rospy.loginfo("======================================")

    def publish_cmd(self, v, omega):
        """
        Publish velocity command to Duckiebot.
        v = linear velocity
        omega = angular velocity
        """
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = v
        msg.omega = omega
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        """
        Stop robot movement.
        """
        self.publish_cmd(0.0, 0.0)

    def clean_shutdown(self):
        """
        Called when node is stopped with Ctrl+C.
        Sends zero velocity to stop robot safely.
        """
        rospy.loginfo("Shutting down target follower. Stopping robot.")
        self.stop_robot()
        rospy.sleep(0.5)

    def tag_callback(self, msg):
        """
        Callback function called whenever AprilTag detections are received.
        """
        self.move_robot(msg.detections)

    def move_robot(self, detections):
        """
        Main control logic for target following.

        Required task behaviours:
        1. If no AprilTag is visible, rotate in place to seek object.
        2. If AprilTag is visible, rotate in place to face the tag.
        """

        # ------------------------------------------------
        # Behaviour 1: Seek object when no tag is detected
        # ------------------------------------------------
        if len(detections) == 0:
            rospy.loginfo("No AprilTag detected: seeking object by rotating")
            self.publish_cmd(0.0, self.seek_omega)
            rospy.sleep(self.command_delay)
            return

        # ------------------------------------------------
        # Behaviour 2: Look at object when tag is detected
        # ------------------------------------------------
        tag = detections[0]

        # AprilTag position in camera coordinate frame
        x = tag.transform.translation.x
        y = tag.transform.translation.y
        z = tag.transform.translation.z

        try:
            tag_id = tag.tag_id
        except AttributeError:
            tag_id = -1

        rospy.loginfo(
            "AprilTag detected: id=%s, x=%.3f, y=%.3f, z=%.3f",
            str(tag_id), x, y, z
        )

        # Use x-position as horizontal error.
        # Goal: drive error to zero, so tag becomes centred.
        error = x

        # If tag is already close to centre, stop rotating.
        if abs(error) < self.dead_zone:
            omega = 0.0
            rospy.loginfo("Tag is centred. Stopping rotation.")

        else:
            # Proportional control
            omega = self.kp * error

            # Apply minimum angular velocity to overcome wheel friction
            if 0.0 < omega < self.min_omega:
                omega = self.min_omega
            elif -self.min_omega < omega < 0.0:
                omega = -self.min_omega

            # Limit maximum angular velocity for safety/stability
            if omega > self.max_omega:
                omega = self.max_omega
            elif omega < -self.max_omega:
                omega = -self.max_omega

            rospy.loginfo("Control error=%.3f, omega=%.3f", error, omega)

        # Important task requirement:
        # Linear velocity must stay zero. Robot only rotates in place.
        self.publish_cmd(0.0, omega)

        rospy.sleep(self.command_delay)


if __name__ == "__main__":
    try:
        node = TargetFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

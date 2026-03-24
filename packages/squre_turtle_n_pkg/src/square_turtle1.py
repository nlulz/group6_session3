#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def draw_square():
    rospy.init_node('square_turtle_node', anonymous=True)
    
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    move_cmd = Twist()

    rospy.loginfo("Turtles are great at drawing squares!")

    while not rospy.is_shutdown():

        # Move forward
        move_cmd.linear.x = 2.0
        move_cmd.angular.z = 0.0

        for _ in range(20):  # adjust for distance
            pub.publish(move_cmd)
            rate.sleep()

        # Stop before turning
        move_cmd.linear.x = 0.0
        pub.publish(move_cmd)
        rospy.sleep(1)

        # Turn 90 degrees
        move_cmd.angular.z = 1.57  # ~90 degrees/sec

        for _ in range(10):  # adjust for angle
            pub.publish(move_cmd)
            rate.sleep()

        # Stop turning
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        draw_square()
    except rospy.ROSInterruptException:
        pass

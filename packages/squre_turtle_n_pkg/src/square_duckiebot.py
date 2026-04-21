#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move(pub, linear_speed, angular_speed, duration):
    vel = Twist()
    vel.linear.x = linear_speed
    vel.angular.z = angular_speed

    rate = rospy.Rate(10)
    start_time = rospy.Time.now().to_sec()

    while rospy.Time.now().to_sec() - start_time < duration and not rospy.is_shutdown():
        pub.publish(vel)
        rate.sleep()

    stop = Twist()
    pub.publish(stop)

def draw_square():
    rospy.init_node('duckiebot_square_node', anonymous=True)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.sleep(2)

    forward_speed = 0.3
    turn_speed = 1.0

    forward_time = 3.0
    turn_time = 1.5

    rospy.loginfo("Starting square movement")

    for i in range(4):
        rospy.loginfo(f"Moving side {i+1}")
        move(pub, forward_speed, 0.0, forward_time)
        rospy.sleep(1)

        rospy.loginfo(f"Turning corner {i+1}")
        move(pub, 0.0, turn_speed, turn_time)
        rospy.sleep(1)

    rospy.loginfo("Finished square")

if __name__ == '__main__':
    try:
        draw_square()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist


PI = 3.14159265359
LIN_VEL = 5
RADIUS = 2


def move():
    # Starts a new node
    rospy.init_node('node_turtle_revolve', anonymous=False)
    velocity_publisher = rospy.Publisher(
        '/turtle1/cmd_vel', Twist, queue_size=10)

    # linear_velocity=angular_velocity*radius
    angular_velocity = (LIN_VEL / RADIUS)

    distance = 2 * PI * RADIUS

    vel_msg = Twist()

    while not rospy.is_shutdown():
        current_distance = 0
        # time in seconds when turtle started moving in the lower circle
        init_time = rospy.Time.now().to_sec()

        while current_distance < 1.1 * distance:  # moving the turtle in the lower circle
            curr_time = rospy.Time.now().to_sec()  # current time in seconds

            vel_msg.linear.x = LIN_VEL
            vel_msg.angular.z = -angular_velocity

           # current_distance = velocity*time
            current_distance = LIN_VEL * (curr_time - init_time)
            velocity_publisher.publish(vel_msg)
            rospy.loginfo("Moving in Circle")

        rospy.loginfo("Reached initial position")
        current_distance = 0
        # time when turtle started moving in the upper circle
        init_time = rospy.Time.now().to_sec()

        while current_distance < .6 * distance:  # moving the turtle in the upper circle
            curr_time = rospy.Time.now().to_sec()   # current time in seconds

            vel_msg.linear.x = LIN_VEL
            vel_msg.angular.z = angular_velocity

            #current_distance = velocity*time
            current_distance = LIN_VEL * (curr_time - init_time)
            velocity_publisher.publish(vel_msg)
            rospy.loginfo("Moving in Circle")

        rospy.loginfo("Reached initial position")
        break


if __name__ == '__main__':

    try:
        move()
    except rospy.ROSInterruptException:
        pass

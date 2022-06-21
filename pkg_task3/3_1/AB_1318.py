#!/usr/bin/env python3
import sys
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import time



class AGV():
    def __init__(self):
        rospy.init_node("ebot_controller", anonymous=False)
        rospy.Subscriber("/ebot/laser/scan", LaserScan, self.laser_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser = LaserScan()
        self.odom_data = Odometry()

        self.regions = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'bleft': 0,
        }
        
        self.count_new = 0

        self.x_init = 0.79999
        self.y_init = -1.35

        self.x_now = 0.0
        self.y_now = 0.0

        self.max_ = 5
        self.yaw = 0.0

        self.count = 0
        self.count2 = 0

        self.case = 0

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.max_radius = 0.35

    def laser_callback(self, laser_msg):
        self.laser = laser_msg
        # dividing laser feedback regions into 5 parts
        self.regions = {
            'bright': min(min(self.laser.ranges[0:143]), self.max_),
            'fright': min(min(self.laser.ranges[144:287]), self.max_),
            'front': min(min(self.laser.ranges[288:431]), self.max_),
            'fleft': min(min(self.laser.ranges[432:576]), self.max_),
            'bleft': min(min(self.laser.ranges[577:719]), self.max_),
        }

    def odom_callback(self, odom):
        self.odom_data = odom
        self.x_now = self.odom_data.pose.pose.position.x
        self.y_now = self.odom_data.pose.pose.position.y
        x = self.odom_data.pose.pose.orientation.x
        y = self.odom_data.pose.pose.orientation.y
        z = self.odom_data.pose.pose.orientation.z
        w = self.odom_data.pose.pose.orientation.w
        euler = euler_from_quaternion([x, y, z, w])
        self.yaw = euler[2]  # Yaw angle for the robot
        self.yaw = self.angle_change(self.yaw)
        # The current coordinate system is changed to cartesian coordinate system
        # and by doing so angles are changed to [0,2*pi]


    def angle_change(self, phi):  # Function used to change angle ranges in the coordinate system
        if phi < 0:
            phi = 2 * math.pi + phi
        return phi

    def laser_feedback(self):         # Function used for detecting obstacles using 2D lidar
        if self.regions['fright'] < 0.65:
            if self.regions['fleft'] > 0.65:
                if self.regions['front'] < 0.65:
                    self.case = 2  # objstacle in front and front right regions
                elif self.regions['front'] > 0.65:
                    self.case = 1   # obstacle in front right region
            elif self.regions['fleft'] < 0.65:
                if self.regions['front'] < 0.65:
                    self.case = 4  # obstacle in front, front right and front left regions
                elif self.regions['front'] > 0.65:
                    self.case = 3  # obstacle in front right and front left regions

        elif self.regions['fright'] > 0.65:
            if self.regions['fleft'] > 0.5:
                if self.regions['front'] > 0.65:
                    if self.regions['bleft'] < 0.65 or self.regions['bright'] < 0.65:
                        self.case = 8  # obstacle only on left or right side
                    else:
                        self.case = 0  # No obstacle detected in any defined regions
                elif self.regions['front'] < 0.65:
                    self.case = 5  # obstacle up front
            elif self.regions['fleft'] < 0.65:
                if self.regions['front'] > 0.65:
                    self.case = 7  # object in front left region
                elif self.regions['front'] < 0.65:
                    self.case = 6  # object in front left and front regions

    def publish(self, l_v, a_v):  # Function used to publish data
        velocity_msg = Twist()
        velocity_msg.linear.x = l_v
        velocity_msg.angular.z = a_v
        self.pub.publish(velocity_msg)

    def move_forward(self):    # Function used to move forward
        self.linear_velocity = 0.4
        self.angular_velocity = 0
        self.publish(self.linear_velocity, self.angular_velocity)

    def stop(self):           # Function used to stop the robot
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.publish(self.linear_velocity, self.angular_velocity)

    def rotate_to_align(self):  # Function used to allign the robot along the trough
        while True:
            if self.regions['bleft'] < 0.7 and self.regions['fleft'] > 0.7:
                break
            self.angular_velocity = -0.5
            self.publish(self.linear_velocity, self.angular_velocity)

    def rotate_left(self, radius):  # Function used to move left
        self.linear_velocity = 0.4
        self.angular_velocity = self.linear_velocity // radius
        self.publish(self.linear_velocity, self.angular_velocity)

    def controller(self):
        while not rospy.is_shutdown():
            self.move_forward()
            self.laser_feedback()

            # count is a flag that increments its value when the robot starts
            # moving

            if self.odom_data.twist.twist.linear.x != 0 and self.count == 0:
                self.count += 1
            while self.count == 1:
                self.move_forward()
                self.laser_feedback()
                # If no trough was found on moving forward then continue moving
                # forward until the robot finds one
                if self.case == 0:
                    print("finding troughs")

                else:
                    # When a wall is found the robot stops and aligns with the
                    # trough
                    self.stop()
                    print("Trough found,and following it")
                    self.rotate_to_align()
                    while True:
                        self.move_forward()
                        # After aligning with the trough the robot moves forward
                        # If the alignment is not parallel to the trough then
                        # the robot aligns itself while moving forward
                        self.rotate_to_align()

                        # At the edge of a trough line, distance feedback from a new region
                        # spanning from line 480 in front left region to line 680 in left region
                        # is stored in a variable called radius.
                        # A max bound is set on this value so that its value wont go to infinity.
                        # When this new region returns a value greater than the set threshold
                        # (most part of the robot has crossed the troughs by moving forward),
                        # Robot rotates to the left with a fixed linear velocity and
                        # angular velocity determined by linear velocity/ radius
                        # ,(radius calculated as mentioned before).

                        # When the value of the variable radius is less than
                        # the set threshold, then the robot moves forward
                        # This loop is continued until the robot rotates around the troughs.
                        # After completing the turn the robot then moves to
                        # trough following state.

                        radius = min(
                            min(self.laser.ranges[480:680]), self.max_radius)
                        while min(
                                min(self.laser.ranges[480:680]), self.max_) > 0.7:
                            self.rotate_left(radius)
                            # The below threshold is given to break from the
                            # first trough and move forward to find the next
                            # one
                            if 0 < self.yaw < 0.1 and self.count2 == 0:
                                self.count2 += 1
                                break
                            # The below threshold is to Stop the robot at its
                            # finish line
                            if self.count2 == 2 and self.y_now > 7.5:
                                self.stop()
                                sys.exit()

                        # The condition to break from trough one and to stop is given twice here.
                        # This is because robot oscillates from
                        # moving forward and rotating left when it finds an edge.
                        # By giving condition twice we can ensure that
                        # the robot follows our desired path.
                        # It is also made sure that if condition at one end is
                        # triggered then condition at other end wont be
                        # triggered

                        # The below threshold is given to break from the first
                        # trough and move forward to find the next one
                        if 0 < self.yaw < 0.1 and self.count2 == 0:
                            self.count2 += 2
                            break

                        if self.count2 == 1:
                            self.count2 += 1
                            break

                        # The below threshold is to Stop the robot at its
                        # finish line
                        if self.count2 == 2 and self.y_now > 7.5:
                            self.stop()
                            sys.exit()

                    # The below part of the code ensures that the robot crosses
                    # first series troughs after breaking from it
                    while self.x_now - self.x_init < 0:
                        self.move_forward()


if __name__ == '__main__':
    try:
        rospy.sleep(60)  # To record bag file fully, waiting for gazebo to open 
        agv = AGV()
        agv.controller()
    except rospy.ROSInterruptException:
        pass

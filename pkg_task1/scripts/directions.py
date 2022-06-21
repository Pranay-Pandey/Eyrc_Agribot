#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

        





def odom_callback(data):
    global pose
    global z_rot_diff
    global z_rot_diff2
    global phase
    
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]
    
    print(pose)

    

def laser_callback(msg):
    global phase
    global z_rot_diff2
    global regions
    regions = {
        'right': msg.ranges[0:40],
        'front':  msg.ranges[319:399]  ,
        'left':  msg.ranges[679:719]  ,
        'right_upper' : msg.ranges[250:300]  ,
        'left_upper' : msg.ranges[418:468]
    }
    # print("right {} , {} ".format(min(regions['right']),max(regions['right'])))
    # # print("front {} , {} ".format(min(regions['front']),max(regions['front'])))
    # print("left {} , {} ".format(min(regions['left']),max(regions['left'])))
    # # print("right_upper {} , {} ".format(min(regions['right_upper']),max(regions['right_upper'])))
    # # print("left_upper {} , {} ".format(min(regions['left_upper']),max(regions['left_upper'])))
  
        



def control_loop():
    rospy.init_node('ebot_controller')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    while not rospy.is_shutdown():

        
        # velocity_msg.angular.z = 
        # pub.publish(velocity_msg)
        # print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()
        

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass




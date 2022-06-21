#!/usr/bin/env python3
from pickle import GLOBAL
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys

PI = 3.14
LIN_VEL = 5
RADIUS = 2
PHASE = 0
x_init = 5.544444561
y_init = 5.544444561
i = 0
global again
again = 0
global magic
magic = 0
vel_msg = Twist()
global done
done = 0
def great():
    print("DONE")
    global done
    done = 1

def positioning(turpose):
    global done
    if(done!=1):
        z = turpose.theta
        print(z)
        if(z==0):
            great()
            vel_msg.linear.x = 0
            vel_msg.angular.z =0
            print("ALIGNED")
            return
        else:
            vel_msg.angular.z = -10*z
            vel_msg.linear.x = 0


def align():
    global done
    
    velocity_publisher3 = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
    subsss = rospy.Subscriber('/turtle1/pose',Pose, positioning)
    
    while not rospy.is_shutdown():
        vel_msg.linear.x = 0
        while True:
            velocity_publisher3.publish(vel_msg)
            subsss
            if(done==1):
                break
        rospy.loginfo("BROKEN 1")
        break
    rospy.loginfo("AFTER BROKEN")
    rospy.loginfo("REACHED INITIAL POS AGAIN")
    return


def t2tune(myPOSs):
    global magic
    global again
    global done
    theta = myPOSs.theta
    if (done!=2 and again == 0):
        rospy.loginfo("Moving in Circle 1")
        if(theta<0 and theta>-0.019):
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            rospy.ROSInterruptException
            rospy.loginfo("Reached initial position")
            magic = 5
        

def pos2andonward():
    vel_msg.linear.x = 1
    vel_msg.angular.z = 1
    PHASE = 2
    global magic
    magic = 1

def stop():
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0

def run2():
    velocity_publisher2 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    distance2 = 2*PI
    while not rospy.is_shutdown():
        alternative_curr_dis2 = 0
        init_time2 = rospy.Time.now().to_sec()
        while alternative_curr_dis2< 0.7 * distance2:
            curr_time2 = rospy.Time.now().to_sec() 
            vel_msg.linear.x = 1
            vel_msg.angular.z = 1
            alternative_curr_dis2 = (curr_time2 - init_time2)
            velocity_publisher2.publish(vel_msg)
        break
    return


def callback(myPOS):
    global PHASE
    global x_init
    global y_init
    global i
    x_current = myPOS.x
    y_current = myPOS.y
    theta = myPOS.theta
    x = myPOS.x
    global magic
    
    if(theta>=0 and theta<0.019):
        i += 1
        if i>0:
            pos2andonward()

def move2():
    global magic
    global done
    global again
    print("MOVE2")
    velocity_publisher2 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    subss = rospy.Subscriber('/turtle1/pose',Pose,t2tune)
    distance = 2*PI
    while not rospy.is_shutdown():
        alternative_curr_dis = 0
        init_time = rospy.Time.now().to_sec()
        while alternative_curr_dis< 0.5 * distance:
            curr_time = rospy.Time.now().to_sec() 
            vel_msg.linear.x = 1
            vel_msg.angular.z = 1
            alternative_curr_dis = (curr_time - init_time)
            velocity_publisher2.publish(vel_msg)
        vel_msg.linear.x = 1
        vel_msg.angular.z = 1
        while True:
            velocity_publisher2.publish(vel_msg)
            subss
            if(magic==5):
                break
        break
    done = 2
    again = 1
    align()
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    return


            


def move():
   
    rospy.init_node('trail1', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    

    
    global PHASE
    global co
    co = 1

    distance = 2*PI

    while not rospy.is_shutdown():

        alternative_curr_dis = 0
        init_time = rospy.Time.now().to_sec()
        while alternative_curr_dis< 0.5 * distance:
            curr_time = rospy.Time.now().to_sec() 
            vel_msg.linear.x = 1
            vel_msg.angular.z = -1
            alternative_curr_dis = (curr_time - init_time)
            velocity_publisher.publish(vel_msg)
            rospy.loginfo("Moving in Circle 2")
        global PHASE
        vel_msg.linear.x = 1
        vel_msg.angular.z = -1
        sub = rospy.Subscriber('/turtle1/pose',Pose,callback)
        
        while True:
            
            velocity_publisher.publish(vel_msg)
            rospy.loginfo("Moving in Circle 3")
            sub

            if(magic==1):
                
                break
            
        rospy.loginfo("Reached initial position")
        break
    align()
    move2()
    
            
            


if __name__ == '__main__':

    try:
        move()
    except rospy.ROSInterruptException:
        pass
    

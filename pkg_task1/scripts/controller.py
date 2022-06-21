#!/usr/bin/env python3
from os import linesep
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

global phase
phase=0
global z_rot_diff
z_rot_diff=0
global z_rot_diff2
z_rot_diff2=0
global flag
flag = 0
global y_ini
y_ini = 0
global x_ini
x_ini = 0
global y_max
y_ini = 0
global flag2
flag2 = 0 
global z_rot_diff3
z_rot_diff3=0  
global line
line=0  
global gospeed
gospeed = 0



def odom_callback(data):
    global pose
    global flag
    global flag2
    global z_rot_diff
    global z_rot_diff2
    global y_ini
    global phase
    global y_max
    global x_ini
    global line
    
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]
    if(flag==0):
        y_ini = pose[1]
        x_ini = pose[0]
        print("x_ini= {}".format(x_ini))
        
        flag = 1

    if(phase==0):
        if(abs(pose[2]-3.2)<0.1):
            phase =1 
        else:
            z_rot_diff = -pose[2]+3.2
            print(abs(pose[2]-3.2))
    elif(phase==2):
        if(abs(pose[2]-1.56)<0.1):
            print("PHASE = 3")
            phase =3
        else:
            z_rot_diff2 = -1.2
            print((pose[2]))
    elif(phase==3):
        line = -pose[2]+1.59
    elif(phase==4):
        if(abs(pose[2])<0.1):
            print("PHASE = 4 complete")
            phase =5
        else:
            z_rot_diff2 = -pose[2]
            print((pose[2]))
    if(phase==5):
        print("{} , {}  ".format(pose[0],x_ini))
        if(abs(pose[0]-x_ini)<0.15):
            y_max = pose[1]
            print("phase =6 now")
            phase = 6            
    if(phase==6):
        if(abs(pose[2]+1.6)<0.1):
            print("PHASE = 6 complete")
            phase =7
        else:
            z_rot_diff2 = -pose[2]-1.70
            print((pose[2]))
    
    if(phase==7):
        # z_rot_diff2 = pose[0] -x_ini
        # print("x_ini = {}".format(x_ini))
        # print("pose[0] = {}".format(pose[0]))
        # z_rot_diff2 = pose[0] -x_ini
        line = 0
        if(pose[1]-y_ini<4):
            line = 1
        if(pose[1]-y_ini<0.5):
            phase = 8
    if(phase==8):
        if(abs(pose[2])<0.1):
            print("PHASE = 8 complete")
            phase =9
        else:
            z_rot_diff2 = -pose[2]
            print((pose[2]))
    if(phase==10):
        if(abs(pose[2]-1.59)<0.1):
            print("PHASE = 10 complete")
            phase =11
        else:
            z_rot_diff2 = -pose[2]+1.59
            print((pose[2]))
    if(phase==11):
        line = -pose[2]+1.59
        if abs(y_max-pose[1])<0.9:
            phase = 12

    

def laser_callback(msg):
    global phase
    global z_rot_diff2
    global z_rot_diff3
    global regions
    global gospeed
    regions = {
        'right': msg.ranges[0:35],
        'front':  msg.ranges[319:399]  ,
        'left':  msg.ranges[694:719]  ,
        'right_upper' : msg.ranges[250:300]  ,#changing phase 3 from right -> right_upper
        'left_upper' : msg.ranges[418:468]
    }
    # # print("right {} , {} ".format(min(regions['right']),max(regions['right'])))
    # # print("front {} , {} ".format(min(regions['front']),max(regions['front'])))
    # # print("left {} , {} ".format(min(regions['left']),max(regions['left'])))
    # # print("right_upper {} , {} ".format(min(regions['right_upper']),max(regions['right_upper'])))
    # # print("left_upper {} , {} ".format(min(regions['left_upper']),max(regions['left_upper'])))
    if(phase==1):
        if((min(regions['front'])<2.4)):
            print("Phase 2 started")
            phase = 2
        print("right {} , {} ".format(min(regions['right']),max(regions['right'])))
    elif(phase==3):
        print((min(regions['right_upper'])))
        z_rot_diff2 = 0
        if(min(regions['right_upper'])<4):
            z_rot_diff2 = min(regions['right_upper']) - 1#(min(regions['left'])) - (min(regions['right']))
            
            if abs(min(regions['right_upper']) - 1)<0.1 or abs(min(regions['right_upper']) - 1)>3:
                z_rot_diff2 = -line
        # if(min(regions['right'])<0.75):
        #     z_rot_diff2 = 0.75 - min(regions['right'])
        # elif(min(regions['right'])>1):
        #     z_rot_diff2 = min(regions['right']) - 0.75
        if((float(min(regions['right']))==float('inf')) or (min(regions['front'])<1) or min(regions['right'])>4):
            print("phase 4")
            z_rot_diff2 = 0
            phase = 4
    if(phase==7):
        print("right {} , {} ".format(min(regions['right']),max(regions['right'])))
        print("left {} , {} ".format(min(regions['left']),max(regions['left'])))
        if(line==1):
            z_rot_diff3 = (min(regions['left'])) - (min(regions['right']))
        else:
            z_rot_diff3 = (min(regions['left_upper'])) - (min(regions['right_upper']))
        # if(z_rot_diff3<0.03):
        #     print("LINE")
        #     z_rot_diff3 = line
    if(phase==9):
        #if(float(min(regions['left'])==float('inf')) or min(regions['left'])>5):
        if(min(regions['left_upper'])>0.6):
            print("phase 9 complete")
            phase = 10
        print("left {} , {} ".format(min(regions['left']),max(regions['left'])))
    if(phase==11):
        z_rot_diff2 = 0
        print(min(regions['left_upper']))
        if(min(regions['left_upper'])<3):
            z_rot_diff2 = min(regions['left_upper']) - 1.3#(min(regions['left'])) - (min(regions['right']))
            gospeed = 0.1
            if abs(min(regions['left_upper']) - 1.3)<0.1 or abs(min(regions['right']) - 1.3)>3:
                z_rot_diff2 = -line
                gospeed = 0.8
        # if((max(regions['left'])<3)):
        #     z_rot_diff2 = (min(regions['left'])) - (min(regions['right']))    
        # if(min(regions['left'])==float('inf')):
        #     phase=12
        

    
        



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

        if(phase==0):
            velocity_msg.angular.z = 1.6*z_rot_diff
            pub.publish(velocity_msg)
            continue
        
        velocity_msg.angular.z = 0
        velocity_msg.linear.x = 0

        if(phase==1):
            velocity_msg.linear.x = 0.6
            velocity_msg.angular.z = 0
            pub.publish(velocity_msg)
        
        if(phase==2):
            velocity_msg.angular.z = -1.7
            velocity_msg.linear.x = 0.5
            pub.publish(velocity_msg)
            continue

        if(phase==3):
            velocity_msg.angular.z = -1.2*z_rot_diff2
            velocity_msg.linear.x = 0.9
            pub.publish(velocity_msg)
            continue
        
        if(phase==4):
            velocity_msg.angular.z =-1.2
            velocity_msg.linear.x = 0.5
            pub.publish(velocity_msg)

        if(phase==5):
            velocity_msg.angular.z = 0
            velocity_msg.linear.x = 0.8
            pub.publish(velocity_msg)

        if(phase==6):
            velocity_msg.angular.z = 1.2*z_rot_diff2
            velocity_msg.linear.x = 0
            pub.publish(velocity_msg)

        if(phase==7):
            velocity_msg.angular.z = 0.75*z_rot_diff3
            velocity_msg.linear.x = 0.9
            pub.publish(velocity_msg)
            
        if(phase==8):
            print("phase 8")
            velocity_msg.angular.z = 1.35*z_rot_diff2
            velocity_msg.linear.x = 0.4
            pub.publish(velocity_msg)

        if(phase==9):
            print("phase 9")
            velocity_msg.angular.z = 0
            velocity_msg.linear.x = 0.6
            pub.publish(velocity_msg)

        if(phase==10):
            print("phase 10")
            velocity_msg.angular.z = 1.6*z_rot_diff2
            velocity_msg.linear.x = 0.2
            pub.publish(velocity_msg)

        if(phase==11):
            
            velocity_msg.angular.z = 0.7*z_rot_diff2
            velocity_msg.linear.x = 0.7
            pub.publish(velocity_msg)

        if(phase==12):
            print("phase 12")
            velocity_msg.angular.z = 0
            velocity_msg.linear.x = 0
            pub.publish(velocity_msg)
            break
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0
        
        rate.sleep()
    return   

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass




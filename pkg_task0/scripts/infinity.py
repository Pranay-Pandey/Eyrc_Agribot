#!/usr/bin/env python3
from re import X
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose



vel_msg = Twist()
x_ini = 0
y_ini = 0
phase = 0






def turtlemove(x,z,ph,Posx,Posy):
	global phase
	global x_ini
	global y_ini
	global msg_vel

	velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

	vel_msg.linear.x = x
	vel_msg.angular.z = z

	while(True):
		velocity_publisher.publish(vel_msg)
		





def callback(myPos):
	global phase
	global x_ini
	global y_ini
	global vel_msg
	ended = 0

	
	if(phase==0 and ended==0):
		ended =1
		x_ini = myPos.x
		y_ini = myPos.y
		print(x_ini,y_ini)
		phase += 1
		return

	


	elif(phase==1 and ended==0):
		ended = 1
		turtlemove(1,-1,phase,myPos.x,myPos.y)
		if(x_ini==myPos.x and y_ini==myPos.y):
			phase +=1 
			return


	elif(phase==2 and ended==0):
		ended= 1
		turtlemove(1,1,phase,myPos.x,myPos.y)
		if(x_ini==myPos.x and y_ini==myPos.y):
			phase +=1 
			return


def move():
	global phase	
	rospy.init_node('infinity',anonymous=False)

	pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose,callback)

	if(phase<3):
		rospy.spin()
	else:
		rospy.loginfo("Reached initial position -> end")
		return

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass

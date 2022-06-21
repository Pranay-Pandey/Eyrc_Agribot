#! /usr/bin/env python3

from numpy.core.records import array
import cv_bridge
import rospy
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import time
import message_filters
import math

l_r = [0, 2, 80]
u_r = [6, 189, 255]
count = 0

center_x = 320.5
center_y = 240.5
f_x = 554.387
f_y = 554.387

count = 0

coordinates = []
coords_img = []
bridge = CvBridge()
pose = geometry_msgs.msg.TransformStamped()
Broadcaster = tf2_ros.TransformBroadcaster()

pub_img = rospy.Publisher("/tomato_identifier/image", Image, queue_size=10)


def main() :
    rospy.init_node("tomato_identifier", anonymous=False)
    img_sub = message_filters.Subscriber("/camera/color/image_raw2", Image)
    depth_sub = message_filters.Subscriber("/camera/depth/image_raw2", Image)

    sub = message_filters.ApproximateTimeSynchronizer([img_sub, depth_sub], 10, 0.001)
    sub.registerCallback(callback)
    
    rospy.spin()


def callback(img_data, depth_data):
    global count
    array_depth = []
    if count == 0 :
        count = 1
        
        # RGB Image
        image = bridge.imgmsg_to_cv2(img_data, "passthrough")     
        
        # Depth Image
        depth_image = bridge.imgmsg_to_cv2(depth_data, "32FC1")   
        
        # Array having depth values, obtained from depth image
        depth_array = np.array(depth_image, dtype=np.float32)     

        # Image is converted to HSV for masking out red colour
        image_hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)          
        img = cv.cvtColor(image, cv.COLOR_RGB2BGR)                
        
        # setting up a 5x5 kernel for erode and dilate functions
        kernel = np.ones((5,5),np.uint8)                          
        
        # creating the mask
        mask = cv.inRange(image_hsv, np.array(l_r), np.array(u_r)) 
        
        # Eroding the mask to remove stray lines
        mask = cv.erode(mask, kernel, iterations=1)               
        
        # Dilating it multiple times :- 
        # 1) to add those pixels of tomatoes eroded 
        # 2) to make a single smooth mask for tomato.
        #    Within masked part of tomato, there are stray lines
        #    which make the entire tomato appear as different contours
        #    for the contour function in openCV. Dilating it will remove
        #    these stray lines and make the mask of the tomato a full contour in itself
        mask = cv.dilate(mask, kernel, iterations=8)              
        contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
        for contour in contours :
            
            # Calculating the center coordinates
            M = cv.moments(contour)
            cx = int(M["m10"]/M["m00"])  
            cy = int(M["m01"]/M["m00"])
            if not math.isnan(depth_array[cy][cx]) :   # Removing nan depth values 
                depth_mm = depth_array[cy][cx]
                if depth_mm < 2:                       # Setting a threshold depth
                    array_depth.append([cx, cy, depth_mm])

            sorted_array = sorted(array_depth,key=lambda x: x[2])    
            for i in range(len(sorted_array)) :
                [c_x, c_y, d] = sorted_array[i]
                cx_world = d*((cx - center_x)/f_x)
                cy_world = d*((cy - center_y)/f_y)
                
                # Printing center on image
                cv.circle(img, (c_x, c_y), 3, [255, 255, 255], -1)   
                
                # putting text at tomatoes center
                cv.putText(img, "obj" + str(len(sorted_array)-1-i), (c_x, c_y), cv.FONT_HERSHEY_COMPLEX, 1, color= [255, 255, 255], thickness=2)
                
                # Publishing the tf
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "camera_depth_frame2"

                pose.child_frame_id = "obj" + str(len(sorted_array) -1 - i)
                pose.transform.translation.x = cx_world
                pose.transform.translation.y = cy_world
                pose.transform.translation.z = d
                
                pose.transform.rotation.x = 0
                pose.transform.rotation.y = 0
                pose.transform.rotation.z = 0
                pose.transform.rotation.w = 1

                Broadcaster.sendTransform(pose)
        coordinates.clear()

        # Publishing the image to a topic
        pub_img.publish(bridge.cv2_to_imgmsg(img, encoding="passthrough"))
        count = 0

if __name__ == "__main__" :
    try :
        main()
    except rospy.ROSInterruptException :
        pass


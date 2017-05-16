#!/usr/bin/env python

# Library includes
import rospy
from std_msgs.msg import UInt16MultiArray
import cv2
import numpy as np

# Setup Publishing topic name
pub = rospy.Publisher('pixel_coordinates', UInt16MultiArray) 

# Start camera
camera_port = 0
camera = cv2.VideoCapture(camera_port)

# Global variables
global image
global prev_index
prev_index = [0,0]

def ProcessImage():

    # Recall Global variables     
    global prev_index 
    global image   

    # Take image from camera    
    ret, img = camera.read()
    image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # Convert to right format
    image_blur = cv2.GaussianBlur(image, (7,7),3)  # Smoothen the image to remove noise
    img_gray = cv2.cvtColor(image_blur, cv2.COLOR_RGB2GRAY) # Convert to gray scale
    
    # Find center of ball    
    circles = cv2.HoughCircles(img_gray,cv2.HOUGH_GRADIENT,1,20,
                                param1=50,param2=30,minRadius=20,maxRadius=30)
    circles = np.uint16(np.around(circles))

    # Draw all the circles that have been found
    for i in circles[0,:]:
            # Draw the outer circle
            cv2.circle(image,(i[0],i[1]),i[2],(0,255,0),2)
            # Draw the center of the circle
            cv2.circle(image,(i[0],i[1]),2,(0,0,255),3)
    
    # Draw boundries where the Laser pointer can follow the ball
    cv2.line(image,(0,402),(342,402),(0,255,255),2)
    cv2.line(image,(1,402),(1,0),(0,255,255),2)
    cv2.line(image,(342,0),(342,402),(0,255,255),2)
    cv2.line(image,(342,1),(0,1),(0,255,255),2)
    cv2.imshow('show', image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        pass

    # Create message for topic to publish
    msg = UInt16MultiArray()
    msg.data = [circles[0,[0]][0][0], circles[0,[0]][0][1]]  # Only 1 circle is assumed, multiple circles not considered
   
    if ( prev_index != msg.data ): # Prevents repeat values from crowding the Buffer

        # Publish the coordinates to pixel_coordinates topic
        pub.publish(msg)        
    else :
	print ("same")

    prev_index = msg.data
    
def myhook():
    print "releasing camera..."
    camera.release() # So that camera can now be used by other applications.
    cv2.destroyAllWindows()

    

if __name__ == '__main__': 

	# Recall global variables
	global image

	rospy.init_node('camera_node') # Setup Node Name

	print("camera node initialized")
	rate = rospy.Rate(21) # 21hz -> limiting speed of the system.
	while not rospy.is_shutdown(): # Wait for CTRL + C
		try:
			ProcessImage()
			rate.sleep()
		except AttributeError: # Catch rint error when no circles found in image
			cv2.line(image,(0,402),(342,402),(0,0,255),2)
			cv2.line(image,(1,402),(1,0),(0,0,255),2)
			cv2.line(image,(342,0),(342,402),(0,0,255),2)
			cv2.line(image,(342,1),(0,1),(0,0,255),2)
			cv2.imshow('show', image)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				pass
			pass
	rospy.on_shutdown(myhook)


 

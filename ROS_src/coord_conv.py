#!/usr/bin/env python

# library includes
import rospy
import numpy as np
import math
from std_msgs.msg import UInt16MultiArray

# Setup Publishing topic name
pub = rospy.Publisher('coord_send',UInt16MultiArray , queue_size=10)  

# callback function when data received from pixel_coordinates 
def callback(data):
    # Read Values
    x = data.data[0]
    y = data.data[1]
    
    # Scale down the value.
    # Camera (0,0) is top left, Servo (0,0) is Pixel (343, 403)
    x = (90.0/1000.0)*(343 - x)  
    y = (90.0/1000.0)*(403 - y)
    
    # Convert to Polar coordinates.
    if((y>= 0) and (x >=0)):
        r = np.sqrt((x*x)+(y*y)) # radius
        th = 90 - math.degrees(math.atan(x/y)) # angle Theta -> (90 -) because Servo (x,y) is Camera (y,x) 
    	coord = UInt16MultiArray() # Topic data type
    	coord.data = [r,th]
    	rate = rospy.Rate(21) # 21hz -> limiting speed of the system.
    	pub.publish(coord) # Publish the topic
    	rate.sleep()
    
def coord_conv():
    
    rospy.init_node('coord_conv') # Setup Node Name
    rospy.Subscriber("pixel_coordinates",UInt16MultiArray , callback) # Topic to subscribe to

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    coord_conv()

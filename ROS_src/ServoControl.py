#!/usr/bin/env python

# library includes
import rospy
import serial as ps
from std_msgs.msg import UInt16MultiArray

# callback function when data received from coord_send 
def callback(data):
    r = data.data[0]
    th = data.data[1]
    
    # Open /dev/ttyUSB0 for Serial communication
    ser = ps.Serial()
    ser.baudrate = 115200 # BAUD Rate
    ser.port = '/dev/ttyUSB0' # Port name
    ser.open() # open

    # Write echo command onto the Gumstix Board to be written into /dev/pwmservo file
    ser.write("echo t"+str(500-(r*(380.0/180.0)))+" >/dev/pwmservo\n")
    ser.write("echo p"+str(500 - (th*(380.0/180.0)))+" >/dev/pwmservo\n")

    # close serial port
    ser.close()
    
def ServoControl():
    
    rospy.init_node('ServoControl') # Setup Node Name
    rospy.Subscriber("coord_send",UInt16MultiArray , callback) # Topic to subscribe to
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
	ServoControl()

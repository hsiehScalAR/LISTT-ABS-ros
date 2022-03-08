#!/usr/bin/env python
import rospy
from lisst_abs_sensor.msg import Turbidity
from std_msgs.msg import Header
import serial
from numpy import random
import serial.tools.list_ports

ports = serial.tools.list_ports.grep('/dev/ttyUSB')

#Automated Scanning of Serial Port for Communication
##################################################################################
for port, desc, hwid in sorted(ports):
        print("Communication available at {}: {} [{}]".format(port, desc, hwid))
print(f'Communicating with port:{port}')
##################################################################################

#Uncomment the below line and set the port to the known port and comment the above three lines
# port = "/dev/ttyUSB0"

#Establish Serial Communication at the Port with:
# BaudRate    = 9600
# ByteSize    = 8 bits
# Parity      = None
# Stopbits    = 1
# FlowControl = None #not catered to in the below line
ser = serial.Serial(port, baudrate=9600,bytesize = serial.EIGHTBITS,parity = serial.PARITY_NONE,stopbits = serial.STOPBITS_ONE )
 
def serial_comm():
    #Publishes on Topic: TurbidityData, msg: Turbidity
    #msg: Turbidity
    # message: 
    ## Header:
    #      seq: (uint32) sequence id strating form 1 and incremented by 1
    #      stamp: (time) returned by rospy.time
    #           secs
    #           nsecs
    #      frame_id: (String) "Turbidity Levels" 
    #  turbidity_mmt: float64
     pub = rospy.Publisher('TurbidityData', Turbidity, queue_size=20)
     rospy.init_node('serial_comm', anonymous=True)
     rate = rospy.Rate(9600) # 10hz
     msg = Turbidity()
     msg.header = Header()
     while not rospy.is_shutdown():
         #Header Timefframe and Turbidity Measurement
         msg.header.seq = msg.header.seq+1
         msg.header.frame_id = "Turbidity Levels"
         msg.header.stamp = rospy.Time.now()
         data = ser.readline().strip().decode()
         if data == '\x00':
             msg.turbidity_mmt = None
         else:
            msg.turbidity_mmt = float(data)
         rospy.loginfo(msg)
         pub.publish(msg)
         rate.sleep()
 
if __name__ == '__main__':
     try:
         serial_comm()
     except rospy.ROSInterruptException:
         pass

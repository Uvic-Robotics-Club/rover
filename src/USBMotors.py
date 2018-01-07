# -*- coding: utf-8 -*-
"""
Created on Sat Jan 06 14:46:14 2018

@author: joell
"""

import rospy
from std_msgs.msg import String
from geometry_msgs import Quaternion
import serial # import Serial Library
import serial.tools.list_ports as list_ports
import string

motors = [None, None, None, None, None, None, None]
motorCallbacks = [None, None, None, None, None, None, None]
motorPublishers = [None, None, None, None, None, None, None]
motorSubscribers = [None, None, None, None, None, None, None]

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

def looper():
    global motors
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        for i in range(len(motors)):
            if motors[i] is None:
                continue
            MotorData = motors[i].readline().strip().split(",")
            for j in range(len(MotorData)):
                MotorData[j] = float(MotorData[j])
            rospy.loginfo( "Motor {} : {}".format(i, MotorData))
        #pub.publish(hello_str)
        rate.sleep()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard {},{},{},{}".format(data.x, data.y, data.z, data.w))

class Callbacks:
    def __init__(self, localArduino=None):
        self.ardu = localArduino

    def Setpoint(self, value):
        try:
            self.ardu.write("s{}".format(value))
            self.ardu.flush()
            rospy.loginfo( "Changing the setpoint to {}".format(value))
        except Exception as e:
            rospy.loginfo( "Setpoint CALLBACK ERROR: ")
            rospy.loginfo( e.message)

    def Kp(self,value):
        try:
            self.ardu.write("p{}".format(value))
            self.ardu.flush()
            rospy.loginfo( "sending Kp value of {}".format(value))

        except Exception as e:
            rospy.loginfo( "Kp CALLBACK ERROR: ")
            rospy.loginfo( e.message)

    def Kd(self,value):
        try:
            self.ardu.write("d{}".format(value))
            self.ardu.flush()
            rospy.loginfo( "sending Kd value of {}".format(value))
        except Exception as e:
            rospy.loginfo( "Kd CALLBACK ERROR: ")
            rospy.loginfo( e.message)

    def Ki(self,value):
        try:
            self.ardu.write("i{}".format(value))
            self.ardu.flush()
            rospy.loginfo( "sending Ki value of {}".format(value))
        except Exception as e:
            rospy.loginfo( "Ki CALLBACK ERROR: ")
            rospy.loginfo( e.message)

if __name__ == '__main__':
    try:
        rospy.loginfo( "Getting all of the available ports");
        rospy.init_node('USBMotors', anonymous=True)
        ports = list(list_ports.comports())
        for (port,name,PID) in ports:
            rospy.loginfo("Testing {} which is port: {}".format(name,port))
            rospy.loginfo( "found the ardunio. opening comms")
            temparduinoData = serial.Serial(port, 9600) #Creating our serial object
            arduinoString = temparduinoData.readline()
            rospy.loginfo(arduinoString.strip())
            if "Motor:" in arduinoString:
                all=string.maketrans('','')
                nodigs=all.translate(all, string.digits)
                arduinoString=arduinoString.translate(all, nodigs)
                MotorNumber = int(arduinoString)
                motors[MotorNumber] = temparduinoData
                motorCallbacks[MotorNumber] = Callbacks(temparduinoData)
                #motorPublishers[MotorNumber] = rospy.Publisher('Motor{}'.format(MotorNumber), String, queue_size=10)
                motorSubscribers[MotorNumber] = rospy.Subscriber('Motor{}'.format(MotorNumber), Quaternion, callback)
            else:
                temparduinoData.close()
        looper()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 06 14:46:14 2018

@author: joell
"""

import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
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
        rospy.loginfo(rospy.get_caller_id() + "I heard {},{},{},{}".format(value.x, value.y, value.z, value.w))
        try:
            self.ardu.write("s{}".format(value.x))
            self.ardu.flush()
            rospy.loginfo( "Changing the setpoint to {}".format(value.x))
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
        rospy.loginfo("Getting all of the available ports")
        DesiredMotor = 1
        if rospy.has_param('~motor'):
            DesiredMotor = rospy.get_param("~motor")
        rospy.loginfo( "Looking for motor {}".format(DesiredMotor))
        rospy.init_node('USBMotors', anonymous=True)
        ports = list(list_ports.comports())
        for (port,name,PID) in ports:
            rospy.loginfo("Testing {} which is port: {}".format(name,port))
            if "USB" in name or "Arduino" in name or "CH340" in name:
                rospy.loginfo( "found the ardunio. opening comms")
            else:
                continue
            temparduinoData = serial.Serial(port, 9600) #Creating our serial object
            arduinoString = temparduinoData.readline()
            rospy.loginfo(arduinoString.strip())
            if "Motor:" in arduinoString:
                all=string.maketrans('','')
                nodigs=all.translate(all, string.digits)
                arduinoString=arduinoString.translate(all, nodigs)
                MotorNumber = int(arduinoString)
                if MotorNumber != DesiredMotor:
                    continue
                motors[MotorNumber] = temparduinoData
                motorCallbacks[MotorNumber] = Callbacks(temparduinoData)
                #motorPublishers[MotorNumber] = rospy.Publisher('Motor{}'.format(MotorNumber), String, queue_size=10)
                motorSubscribers[MotorNumber] = rospy.Subscriber('Motor{}'.format(MotorNumber), Quaternion, motorCallbacks[MotorNumber].Setpoint)
            else:
                temparduinoData.close()
        looper()
    except rospy.ROSInterruptException:
        pass
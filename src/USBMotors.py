#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 06 14:46:14 2018

@author: joell
"""

import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
import serial # import Serial Library
import serial.tools.list_ports as list_ports
import string

motors = None
motorCallbacks = None
motorPublisher = None
motorSubscriber = None

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
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if(motors.in_waiting == 0):
            MotorData = motors.readline().strip().split(",")
            motorPublisher.publish(MotorData)
            for j in range(len(MotorData)):
                MotorData[j] = float(MotorData[j])
            #rospy.loginfo("{} : {}".format(rospy.get_caller_id(), MotorData))
        rate.sleep()
    motors.close()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard {},{},{},{}".format(data.x, data.y, data.z, data.w))

class Callbacks:
    def __init__(self, localArduino=None):
        self.ardu = localArduino

    def Setpoint(self, value):
        try:
            self.ardu.write("s{}".format(value.data))
            self.ardu.flush()
            rospy.loginfo( "Changing the setpoint to {}".format(value.data))
        except Exception as e:
            rospy.loginfo( "Setpoint CALLBACK ERROR: ")
            rospy.loginfo( e.message)

    def Kp(self,value):
        try:
            self.ardu.write("p{}".format(value.data))
            self.ardu.flush()
            rospy.loginfo( "sending Kp value of {}".format(value.data))

        except Exception as e:
            rospy.loginfo( "Kp CALLBACK ERROR: ")
            rospy.loginfo( e.message)

    def Kd(self,value):
        try:
            self.ardu.write("d{}".format(value.data))
            self.ardu.flush()
            rospy.loginfo( "sending Kd value of {}".format(value.data))
        except Exception as e:
            rospy.loginfo( "Kd CALLBACK ERROR: ")
            rospy.loginfo( e.message)

    def Ki(self,value):
        try:
            self.ardu.write("i{}".format(value.data))
            self.ardu.flush()
            rospy.loginfo( "sending Ki value of {}".format(value.data))
        except Exception as e:
            rospy.loginfo( "Ki CALLBACK ERROR: ")
            rospy.loginfo( e.message)
    def RefreshRate(self,value):
        try:
            self.ardu.write("r{}".format(value.data))
            self.ardu.flush()
            rospy.loginfo( "sending RefreshRate value of {}".format(value.data))
        except Exception as e:
            rospy.loginfo( "RefreshRate CALLBACK ERROR: ")
            rospy.loginfo( e.message)

if __name__ == '__main__':
    try:
        rospy.init_node('USBMotors', anonymous=True)
        rospy.loginfo("Getting all of the available ports")
        DesiredMotor = 1
        if rospy.has_param('~motor'):
            DesiredMotor = int(rospy.get_param("~motor"))
            rospy.loginfo("Looking for motor {}".format(DesiredMotor))

        if rospy.has_param('~port'):
            DesiredPort = rospy.get_param("~port")
            rospy.loginfo("Setting my port to {} and the motor number to {}".format(DesiredPort, DesiredMotor))
            motors = serial.Serial(DesiredPort, 9600)
            arduinoString = motors.readline()
            rospy.loginfo(arduinoString.strip())
            if "Motor:" in arduinoString:
                all=string.maketrans('','')
                nodigs=all.translate(all, string.digits)
                arduinoString=arduinoString.translate(all, nodigs)
                MotorNumber = int(arduinoString)

            motorCallbacks = Callbacks(motors)
            motorPublisher = rospy.Publisher('Motor{}'.format(MotorNumber), String, queue_size=10)
            motorSubscribers = rospy.Subscriber('Motor{}/Setpoint'.format(MotorNumber), Int32, motorCallbacks.Setpoint)
            motorSubscribers = rospy.Subscriber('Motor{}/Kp'.format(MotorNumber), Int32, motorCallbacks.Kp)
            motorSubscribers = rospy.Subscriber('Motor{}/Ki'.format(MotorNumber), Int32, motorCallbacks.Ki)
            motorSubscribers = rospy.Subscriber('Motor{}/Kd'.format(MotorNumber), Int32, motorCallbacks.Kd)
            motorSubscribers = rospy.Subscriber('Motor{}/RefreshRate'.format(MotorNumber), Int32, motorCallbacks.RefreshRate)

            looper()
            rospy.spin()


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
                motors = temparduinoData
                motorCallbacks = Callbacks(temparduinoData)
                #motorPublishers[MotorNumber] = rospy.Publisher('Motor{}'.format(MotorNumber), String, queue_size=10)
                motorSubscribers = rospy.Subscriber('Motor{}/Setpoint'.format(MotorNumber), Int32, motorCallbacks.Setpoint)
                motorSubscribers = rospy.Subscriber('Motor{}/Kp'.format(MotorNumber), Int32, motorCallbacks.Kp)
                motorSubscribers = rospy.Subscriber('Motor{}/Ki'.format(MotorNumber), Int32, motorCallbacks.Ki)
                motorSubscribers = rospy.Subscriber('Motor{}/Kd'.format(MotorNumber), Int32, motorCallbacks.Kd)
                motorSubscribers = rospy.Subscriber('Motor{}/RefreshRate'.format(MotorNumber), Int32, motorCallbacks.RefreshRate)
            else:
                temparduinoData.close()
        looper()
    except rospy.ROSInterruptException:
        pass
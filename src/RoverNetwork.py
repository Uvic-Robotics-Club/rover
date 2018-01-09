#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 08 20:35:02 2018

@author: joell
"""
import rospy
import socket
import sys
import json

import time
import threading

from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point

totalExit = False


def readStuff():
    global totalExit,camera_quality
    sys.stdout.write("Start of receiving thread\n")
    sys.stdout.flush()
    HOST = ''
    try:
        sys.stdout.write("LOOKING FOR DESKTOP\n")
        sys.stdout.flush()
        HOST = socket.gethostbyname('Joel-Desktop')
    except Exception as e:
        print e
    if HOST == '':
        try:
            sys.stdout.write("LOOKING FOR LAPTOP\n")
            sys.stdout.flush()
            HOST = socket.gethostbyname('DESKTOP-VSET45C')
        except Exception as e:
            print e
    if(HOST==""):
        totalExit = True
        print 'Could not find the right computers on the network, EXITING'
        return 1

    PORT = 314
    loop1 = True
    sys.stdout.write("about to connect to the server\n")
    sys.stdout.flush()
    while loop1 and not totalExit:
        loop2 = True
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        sys.stdout.write("connected to server - receiver\n")
        sys.stdout.flush()
        o_data = ''
        while loop2 and not totalExit:
            try:
                data = s.recv(1024)
                # new format will be [m0,m1,m2,m3] with speeds from [-255,255]
                if(not data == o_data):
                    ndata = {}
                    ndata = json.loads(data)
                    if(ndata.has_key('exit')):
                        totalExit = True
                    elif(ndata.has_key('width')):
                        pass
                    elif(ndata.has_key('height')):
                       pass
                    elif(ndata.has_key('quality')):
                        pass
                    else:
                        dictMoveMotors(ndata)
                o_data = data
            except ValueError as e:
                #print "caught value exception in client. The error is: "+e.message
                pass
            except Exception as e:
                print "caught general exception in reading client which is: |" + e.message +"|"
                loop2 = False
                break
        s.close()
    return


def map(value,linput,uinput,loutput,uoutput):
    temp = ((value-linput)*(uoutput-loutput)/(uinput-linput)+loutput)
    if(temp>uoutput):
        temp = uoutput
    elif(temp<loutput):
        temp = loutput
    return temp


def dictMoveMotors(receivedData):
    for key in receivedData:
        receivedData[key] = int(receivedData[key])
    for key in receivedData:
        sys.stdout.write("TRYING TO PUBLISH {} to {}".format(receivedData[key],key))
        motors[key].publish("{}".format(receivedData[key]))


if(__name__=="__main__"):
    rospy.init_node('RoverNetwork', anonymous=True)
    motors={"M1":rospy.Publisher("Motor1/Setpoint", Int32, queue_size=10),
            "M2":rospy.Publisher("Motor2/Setpoint", Int32, queue_size=10),
            "M3":rospy.Publisher("Motor3/Setpoint", Int32, queue_size=10),
            "M4":rospy.Publisher("Motor4/Setpoint", Int32, queue_size=10)}
    threads = []
    t2 = threading.Thread(target=readStuff)

    threads.append(t2)

    for thread in threads:
        time.sleep(0.2)
        thread.start()


    while True:
        if(threads[0].isAlive()):
            sys.stdout.write("still alive ..." + str(time.ctime())+"\r")
            sys.stdout.flush()
            try:
                time.sleep(1)
            except Exception as e:
                totalExit = True
                print 'Trying to exit'
        else:
            break


    print "exiting safely"

    #while True:
    #    readStuff()


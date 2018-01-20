# -*- coding: utf-8 -*-
"""
Created on Mon May 22 22:53:55 2017
TODO: when on the pi, it is difficult to exit out of all the threads at once. Create a safeguard.
@author: joell
"""
import socket
import sys
import json
import math
import atexit
import time
import threading
import numpy as np
import cv2

totalExit = False

    # Captures a single image from the camera and returns it in PIL format
def get_image():
    # read is the easiest way to get a full image out of a VideoCapture object.
    retval, im = camera.read()
    return im

def readStuff():
    global totalExit,camera_quality,HOST
    sys.stdout.write("Start of receiving thread\n")
    sys.stdout.flush()

    PORT = 314
    loop1 = True
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
                    print ndata
                    if(ndata.has_key('exit')):
                        totalExit = True
                    elif(ndata.has_key('width')):
                        camera.set(3,int(ndata['width']))
                    elif(ndata.has_key('height')):
                        camera.set(4,int(ndata['height']))
                    elif(ndata.has_key('quality')):
                        camera_quality = int(ndata['quality'])
                    else:
                    #ndata = eval(ndata)
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

def sendStuff():
    global totalExit, HOST

    sys.stdout.write("Start of sending thread\n")
    sys.stdout.flush()

    PORT = 628
    loop1 = True
    while loop1 and not totalExit:
        loop2 = True
        sys.stdout.write("connecting to server - sender\n")
        sys.stdout.flush()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sys.stdout.write("socket is created, trying to connect to {}::{} - sender\n".format(HOST,PORT))
        sys.stdout.flush()
        s.connect((HOST, PORT))
        sys.stdout.write("connected to server - sender\n")
        sys.stdout.flush()
        while loop2 and not totalExit:
            try:
                s.send("new frame")
                frame = get_image()
                encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),camera_quality]
                result, imgencode = cv2.imencode('.jpg', frame, encode_param)
                data = np.array(imgencode)
                stringData = data.tostring()

                s.send( str(len(stringData)).ljust(16));
                s.send( stringData );
    			# Tell the listener that all pieces have been sent
                s.send("Image Complete")

                s.send(time.ctime())
                time.sleep(0)

            except Exception as e:
                if(e.message != ""):
                    print "caught general exception in sending which is: |" + e.message+"|"
                    #totalExit = True
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
        #print "{}:{}".format(key, receivedData)
        pass


if(__name__=="__main__"):
    # Camera 0 is the usb webcam
    camera_port = 0


    # Now we can initialize the camera capture object with the cv2.VideoCapture class.
    # http://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html#videocapture-set
    # All it needs is the index to a camera port.
    camera = cv2.VideoCapture(camera_port)
    camera.set(3,854) #set the width
    camera.set(4,480) #set the height
    camera_quality = 90

    if(camera==None):
        print "EXITING because I cant find any webcam!"
        sys.exit()

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
        sys.exit()


    motors = {"M1":1,"M2":2,"M3":3,"M4":4}
    motorDirs = {"M1":0,"M2":0,"M3":0,"M4":0}

    threads = []
    t1 = threading.Thread(target=sendStuff)
    t2 = threading.Thread(target=readStuff)

    threads.append(t2)
    threads.append(t1)

    for thread in threads:
        time.sleep(0.2)
        thread.start()


    while True:
        if(threads[0].isAlive() and threads[1].isAlive()):
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


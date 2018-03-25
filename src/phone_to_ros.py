#!/usr/bin/env python

'''
This node is designed to work with this app on an android phone
https://play.google.com/store/apps/details?id=com.ianovir.hyper_imu

Sensor 1:
    Acceleration
        x
        y
        z
Sensor 2:
    Magnetic
        x
        y
        z
Sensor 3:
    Gyroscope
        x
        y
        z
Sensor 4:
    Barometer
        x
        y = 0
        z = 0
Sensor 5:
    Rotation Vector
        x
        y
        z
Sensor 6:
    Gravity
        x
        y
        z
Sensor 7:
    Linear Acceleration
        x
        y
        z
Sensor 8:
    GPS
        lat
        long
        alt

'''
import rospy
import socket, traceback
import numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def mapValue(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;



def split_sensors(raw_message):
    split_message = [x.strip() for x in raw_message.split(',')]
    split_message = np.reshape(split_message, (-1,3))
    #rospy.loginfo(split_message)

    msg = Vector3()
    for sensor_topic in topics:
        if sensor_topic is "Magnetic":
            continue
        sensor_location = topics[sensor_topic]
        msg.x = float(split_message[sensor_location][0])
        msg.y = float(split_message[sensor_location][1])
        msg.z = float(split_message[sensor_location][2])
        sensorPub[sensor_topic].publish(msg)

    sat_msg = NavSatFix()
    sat_msg.latitude = float(split_message[-1][0])
    sat_msg.longitude = float(split_message[-1][1])
    sat_msg.altitude = float(split_message[-1][2])
    sensorPub["GPS"].publish(sat_msg)


    ax = float(split_message[topics["G_Rotation"]][0])
    ay = float(split_message[topics["G_Rotation"]][1])
    az = float(split_message[topics["G_Rotation"]][2])

    pitch = mapValue(ax,-1.0,1.0,-np.pi,np.pi)
    roll = mapValue(ay,-1.0,1.0,-np.pi,np.pi)
    yaw= mapValue(az,-1.0,1.0,-np.pi,np.pi)

    quat = quaternion_from_euler(roll,pitch,yaw)



    imu_msg = Imu()
    imu_msg.header.frame_id = "map"
    imu_msg.header.stamp = rospy.get_rostime()

    imu_msg.orientation.x = quat[0]
    imu_msg.orientation.y = quat[1]
    imu_msg.orientation.z = quat[2]
    imu_msg.orientation.w = quat[3]

    imu_msg.angular_velocity.x =float(split_message[topics["LinearAcceleration"]][0])
    imu_msg.angular_velocity.y =float(split_message[topics["LinearAcceleration"]][1])
    imu_msg.angular_velocity.z =float(split_message[topics["LinearAcceleration"]][2])

    imu_msg.linear_acceleration.x = float(split_message[topics["Gyroscope"]][0])
    imu_msg.linear_acceleration.y = float(split_message[topics["Gyroscope"]][1])
    imu_msg.linear_acceleration.z = float(split_message[topics["Gyroscope"]][2])
    sensorPub["RVIZ_IMU"].publish(imu_msg)

    mag_msg = MagneticField()
    mag_msg.header.frame_id = "map"
    mag_msg.header.stamp = rospy.get_rostime()
    mag_msg.magnetic_field.x = float(split_message[topics["Magnetic"]][0])
    mag_msg.magnetic_field.y = float(split_message[topics["Magnetic"]][1])
    mag_msg.magnetic_field.z = float(split_message[topics["Magnetic"]][2])
    #mag_msg.magnetic_field_covariance = 0
    sensorPub["Magnetic"].publish(mag_msg)


    rviz_msg = PoseStamped()
    rviz_msg.header.frame_id = "map"
    rviz_msg.header.stamp = rospy.get_rostime()
    rviz_msg.pose.position.x = 0
    rviz_msg.pose.position.y = 0
    rviz_msg.pose.position.z = 0
    rviz_msg.pose.orientation.x = quat[0]
    rviz_msg.pose.orientation.y = quat[1]
    rviz_msg.pose.orientation.z = quat[2]
    rviz_msg.pose.orientation.w = quat[3]

    sensorPub["RVIZ"].publish(rviz_msg)

def udp_receive():
    host = ''
    port = 5555

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind((host, port))



    while not rospy.is_shutdown():
        try:
            message, address = s.recvfrom(8192)
            #rospy.logwarn(message)
            split_sensors(message)
        except:
            traceback.print_exc()

if __name__ == '__main__':
    rospy.init_node('phone2ros', anonymous=True)
    sensorPub = {}
    sensorPub["Acceleration"] = rospy.Publisher("Sensor/Acceleration",Vector3, queue_size = 1)
    sensorPub["Magnetic"] = rospy.Publisher("imu/mag",MagneticField, queue_size = 1)
    sensorPub["Gyroscope"] = rospy.Publisher("Sensor/Gyroscope",Vector3, queue_size = 1)
    sensorPub["Barometer"] = rospy.Publisher("Sensor/Barometer",Vector3, queue_size = 1)
    sensorPub["Rotation"] = rospy.Publisher("Sensor/Rotation",Vector3, queue_size = 1)
    sensorPub["Gravity"] = rospy.Publisher("Sensor/Gravity",Vector3, queue_size = 1)
    sensorPub["LinearAcceleration"] = rospy.Publisher("Sensor/LinearAcceleration",Vector3, queue_size = 1)
    sensorPub["G_Rotation"] = rospy.Publisher("Sensor/GameRotation",Vector3, queue_size = 1)
    sensorPub["GPS"] = rospy.Publisher("Sensor/GPS",NavSatFix, queue_size = 1)
    sensorPub["RVIZ"] = rospy.Publisher("Sensor/Pose_test",PoseStamped, queue_size = 1)
    sensorPub["RVIZ_IMU"] = rospy.Publisher("imu/data_raw",Imu, queue_size = 1)

    topics = {"Acceleration":0,"Magnetic":1,"Gyroscope":2,"Barometer":3,"G_Rotation":4,"Rotation":5,"Gravity":6,"LinearAcceleration":7}



    udp_receive()

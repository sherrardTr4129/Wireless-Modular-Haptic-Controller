#!/usr/bin/python

# Author: Trevor Sherrard
# Course: Directed Research
# Since: July 25rd, 2021
# Description: This file instantiates a node that reads from 
#              the RX arduino and publishes quaternion data
#              to the rest of the system

import rospy
import time
import math
import json
from SerialManagerClass import SerialManager 
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

# define global variables
bno055Topic = "/bno055_quat"
port = "/dev/ttyUSB0"
baud = 9600
serObj = SerialManager(port, baud)
sleepTime = 1

class hapticControllerDriver:
    def __init__(self):
        """
        class constructor
        """
        self.bno055QuatTopic = "/bno055_quat"
        self.bno055EulerTopic = "/bno055_euler"
        self.quatPub = None
        self.eulerPub = None
        self.port = "/dev/ttyUSB1"
        self.baud = 9600
        self.serObj = SerialManager(self.port, self.baud)
        self.sleepTime = 1

    def procSerialData(self, RxText):
        """
        This function extracts the individual euler angle components from the recieved
        serial text. The individual components are packed into a dictionary and returned
        for processing

        params:
            RxText (str): The string returned from the serial port
        returns:
            eulerDict (dict): a dictionary representation of the extracted euler data.
                             the dictionary will be empty if data extraction fails
            status (bool): a boolean indicating the status of the data extraction
        """
        eulerDict = dict()

        try:
            # de-serialize json
            YPRdict = json.loads(RxText)

            # return new dict
            return True, YPRdict
        except ValueError:
            return False, dict()

    def publishing_thread(self):
        """
        this function acts as a publishing thread for BNO055
        data comming in from the controller.
        """
        while(not rospy.is_shutdown()):
            # grab line and process
            lineToProc = self.serObj.read()
            if(len(lineToProc) == 0):
                continue
            else:
                status, rpyDict = self.procSerialData(lineToProc)
                if(status):
                    try:
                        # extract rpy data
                        rospy.loginfo(rpyDict)
                        roll = float(rpyDict["euler_y"])
                        pitch = float(rpyDict["euler_z"])
                        yaw = float(rpyDict["euler_x"])

                        # invert pitch due to reversed sensor mounting
                        pitch = -1*pitch

                        # constuct vector3 message
                        rpy = Vector3Stamped()
                        rpy.header.stamp = rospy.Time.now()
                        rpy.vector.x = yaw
                        rpy.vector.y = roll
                        rpy.vector.z = pitch

                        # publish data
                        self.eulerPub.publish(rpy)

                        # convert to radians
                        roll = roll * (math.pi/180)
                        pitch = pitch * (math.pi/180)
                        yaw = yaw * (math.pi/180)

                        # back to quaternion
                        newQuat = quaternion_from_euler(roll, pitch, yaw)

                        # construct Quaternion message
                        quatMessage = QuaternionStamped()
                        quatMessage.header.stamp = rospy.Time.now()
                        qX = newQuat[0]
                        qY = newQuat[1]
                        qZ = newQuat[2]
                        qW = newQuat[3]

                        quatMessage.quaternion.x = qX
                        quatMessage.quaternion.y = qY
                        quatMessage.quaternion.z = qZ
                        quatMessage.quaternion.w = qW

                        # publish quat data
                        self.quatPub.publish(quatMessage)
                    except ValueError, err:
                        rospy.logerr("could not extract all data from serial string: %s" % err)
                        continue

    def init_driver(self):
        # init node
        rospy.init_node("haptic_controller_ros_driver")
        rospy.loginfo("bno055_ros_driver_node initialized")

        # create publisher objects
        self.quatPub = rospy.Publisher(self.bno055QuatTopic, QuaternionStamped, queue_size=1)
        self.eulerPub = rospy.Publisher(self.bno055EulerTopic, Vector3Stamped, queue_size=1)

        # try to open serial port
        res = self.serObj.open()

        # wait a second before procedeing to make sure
        # that serial port has been set up
        time.sleep(self.sleepTime)

        if(not res):
            rospy.logerr("could not open serial device! Double check port and baud settings ")
            return -1

        # start publishing data
        else:
            rospy.logerr("opened serial port: %s at baudrate: %s" % (self.port, self.baud))
            self.publishing_thread()

if __name__ == "__main__":
    try:
        haptic_drv = hapticControllerDriver()
        haptic_drv.init_driver()

    except rospy.ROSInterruptException:
        pass

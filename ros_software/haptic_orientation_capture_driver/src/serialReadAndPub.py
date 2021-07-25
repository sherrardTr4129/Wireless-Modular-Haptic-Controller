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
from geometry_msgs.msg import QuaternionStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

# define global variables
bno055Topic = "/bno055_quat"
port = "/dev/ttyUSB0"
baud = 9600
serObj = SerialManager(port, baud)
sleepTime = 1

def procSerialData(RxText):
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

        # unpack data
        eulerDict["R"] = YPRdict["euler_x"]
        eulerDict["Y"] = YPRdict["euler_y"]
        eulerDict["P"] = -1*YPRdict["euler_z"]

        # return new dict
        return True, eulerDict
    except ValueError:
        return False, dict()

def main():
    # init node
    rospy.init_node("bno055_ros_driver")
    rospy.loginfo("bno055_ros_driver_node initialized")

    # create publisher object
    quatPub = rospy.Publisher(bno055Topic, QuaternionStamped, queue_size=1)

    # try to open serial port
    res = serObj.open()

    # wait a second before procedeing to make sure
    # that serial port has been set up
    time.sleep(sleepTime)

    if(not res):
        rospy.logerr("could not open serial device! Double check port and baud settings ")
        return -1

    # start publishing data
    else:
        while(not rospy.is_shutdown()):
            # grab line and process
            lineToProc = serObj.read()
            if(len(lineToProc) == 0):
                continue
            else:
                status, rpyDict = procSerialData(lineToProc)
                if(status):
                    # extract rpy data
                    roll = float(rpyDict["R"])
                    pitch = float(rpyDict["P"])
                    yaw = float(rpyDict["Y"])

                    # convert to radians
                    roll = roll * (math.pi/180)
                    pitch = -1*pitch * (math.pi/180)
                    yaw = -1*yaw * (math.pi/180)

                    # back to quaternion
                    newQuat = quaternion_from_euler(roll, pitch, yaw)

                    try:
                        # construct Quaternion message
                        quatMessage = QuaternionStamped()
                        quatMessage.header.stamp = rospy.Time.now()
                        qX = newQuat[0]
                        qY = newQuat[1]
                        qZ = newQuat[2]
                        qW = newQuat[3]

                        # make sure to swap x and z axis
                        quatMessage.quaternion.z = qX
                        quatMessage.quaternion.y = qY
                        quatMessage.quaternion.x = qZ
                        quatMessage.quaternion.w = qW

                        # publish data
                        quatPub.publish(quatMessage)
                    except ValueError, err:
                        rospy.logerr("could not extract all quaternion components from serial string: %s" % err)
                        continue
        
        # close the serial port
        serObj.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

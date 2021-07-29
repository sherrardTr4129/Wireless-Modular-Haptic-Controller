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
import sys
from SerialManagerClass import SerialManager 
from haptic_orientation_capture_driver.msg import bno055_euler_data, bno055_quat_data
from haptic_orientation_capture_driver.msg import controller_event_data, controller_haptic_voice
from haptic_orientation_capture_driver.srv import haptic_voice_event
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

class hapticControllerDriver:
    def __init__(self):
        """
        class constructor
        """
        self.bno055QuatTopic = "/bno055_quat"
        self.bno055EulerTopic = "/bno055_euler"
        self.buttonEventTopic = "/controller_button_event"
        self.hapticVoiceEventSrv = "/haptic_voice_event"
        self.quatPub = None
        self.eulerPub = None
        self.eventPub = None
        self.port = ""
        self.baud = 9600
        self.serObj = SerialManager(self.port, self.baud)
        self.sleepTime = 1

    def determinePacketType(self, RxText):
        """
        This function determines the type of event recieved
        from a given controller.

        params:
            RxText (str): The string returned from the serial port
        returns:
            eventType (str): either button_event or outbound_data
        """
        try:
            # de-serialize json
            totalDict = json.loads(RxText)
            eventType = totalDict['packet_type']
            return eventType

        except ValueError:
            return ""

    def procSerialData(self, RxText):
        """
        This function extracts the individual data components from the recieved
        serial text. The individual components are packed into a dictionary and returned
        for processing

        params:
            RxText (str): The string returned from the serial port
        returns:
            dataDict (dict): a dictionary representation of the extracted data.
                             the dictionary will be empty if data extraction fails
            status (bool): a boolean indicating the status of the data extraction
        """
        try:
            # de-serialize json
            dataDict = json.loads(RxText)

            # return new dict
            return True, dataDict
        except ValueError:
            return False, dict()

    def publishingThread(self):
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
                # determine packet type
                packetType = self.determinePacketType(lineToProc)

                # if it's a button event, publish to system
                if(packetType == "button_event"):
                    # unpack data
                    status, dataDict = self.procSerialData(lineToProc)
                    if(status):
                        # create controller event message
                        controllerEvent = controller_event_data()
                        controllerEvent.header.stamp = rospy.Time.now()
                        controllerEvent.controller_name = dataDict["controller_id"]
                        controllerEvent.event_type = dataDict["event_type"]

                        # publish to topic
                        self.eventPub.publish(controllerEvent)

                # if it's a data packet, unpack and publish
                elif(packetType == "outbound_data"):
                    status, rpyDict = self.procSerialData(lineToProc)
                    if(status):
                        try:
                            # extract controller name
                            controllerName = rpyDict["controller_id"]

                            # extract rpy data
                            roll = float(rpyDict["euler_y"])
                            pitch = float(rpyDict["euler_z"])
                            yaw = float(rpyDict["euler_x"])

                            # invert pitch due to reversed sensor mounting
                            pitch = -1*pitch

                            # construct vector3 message
                            rpy = bno055_euler_data()
                            rpy.controller_name = controllerName
                            rpy.header.stamp = rospy.Time.now()
                            rpy.yaw = yaw
                            rpy.roll = roll
                            rpy.pitch = pitch

                            # publish data
                            self.eulerPub.publish(rpy)

                            # convert to radians
                            roll = roll * (math.pi/180)
                            pitch = pitch * (math.pi/180)
                            yaw = yaw * (math.pi/180)

                            # back to quaternion
                            newQuat = quaternion_from_euler(roll, pitch, yaw)

                            # construct Quaternion message
                            quatMessage = bno055_quat_data()
                            quatMessage.controller_name = controllerName
                            quatMessage.quat.header.stamp = rospy.Time.now()
                            qX = newQuat[0]
                            qY = newQuat[1]
                            qZ = newQuat[2]
                            qW = newQuat[3]

                            quatMessage.quat.quaternion.x = qX
                            quatMessage.quat.quaternion.y = qY
                            quatMessage.quat.quaternion.z = qZ
                            quatMessage.quat.quaternion.w = qW

                            # publish quat data
                            self.quatPub.publish(quatMessage)
                        except ValueError, err:
                            rospy.logerr("could not extract all data from serial string: %s" % err)
                            continue

    def hapticVoiceEvent(self, req):
        """
        This function serves as the haptic and voice control 
        ROS service for the haptic controller network.

        params:
            req (haptic_voice_event instance): the request to 
                                               the service
        return:
            None
        """
        # extract request information
        voice_action_id = req.haptic_voice_event.voice_action_id
        haptic_action_id = req.haptic_voice_event.haptic_action_id
        controller_id = req.haptic_voice_event.controller_name

        # create dictionary
        dict_to_send = dict()
        dict_to_send["controller_name"] = controller_id
        dict_to_send["voice_action_id"] = voice_action_id
        dict_to_send["haptic_action_id"] = haptic_action_id

        # serialize JSON
        json_text = json.dumps(dict_to_send)
        self.serObj.write(json_text)

        return True

    def initDriver(self):
        # get port from cmd line args
        args = rospy.myargv(argv=sys.argv)
        if len(args) < 2:
            rospy.logerr("include xbee_port argument")
            return -1
        else:
            self.port = str(args[1])

        # init node
        rospy.init_node("haptic_controller_ros_driver")
        rospy.loginfo("bno055_ros_driver_node initialized")

        # create publisher objects
        self.quatPub = rospy.Publisher(self.bno055QuatTopic, bno055_quat_data, queue_size=1)
        self.eulerPub = rospy.Publisher(self.bno055EulerTopic, bno055_euler_data, queue_size=1)
        self.eventPub = rospy.Publisher(self.buttonEventTopic, controller_event_data, queue_size=1)

        # create rosservice handlers
        rospy.Service(self.hapticVoiceEventSrv, haptic_voice_event, self.hapticVoiceEvent)

        # try to open serial port
        self.serObj = SerialManager(self.port, self.baud)
        res = self.serObj.open()

        # wait a second before procedeing to make sure
        # that serial port has been set up
        time.sleep(self.sleepTime)

        if(not res):
            rospy.logerr("could not open serial device! Double check port and baud settings ")
            return -1

        # start publishing data
        else:
            rospy.loginfo("opened serial port: %s at baudrate: %s" % (self.port, self.baud))
            self.publishingThread()

if __name__ == "__main__":
    try:
        haptic_drv = hapticControllerDriver()
        haptic_drv.initDriver()

    except rospy.ROSInterruptException:
        pass

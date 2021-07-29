#!/usr/bin/env python

# Author: Trevor Sherrard
# Since: July 28, 2021
# Purpose: This node translates the current orientation
#          of the haptic controller into robot base motion

import rospy
from pyrobot import Robot
import time
import numpy as np
from sensor_msgs.msg import LaserScan
from haptic_orientation_capture_driver.msg import bno055_euler_data
from haptic_orientation_capture_driver.msg import controller_event_data, controller_haptic_voice
from haptic_orientation_capture_driver.srv import haptic_voice_event

class hapticBaseControl:
    def __init__(self, controller_id):
        self.nodeName = "haptic_base_control_node"
        
        # define controller id
        self.controller_id = controller_id

        # define locobot instance
        self.LoCoBotInstance = None

        # define placeholder for euler data
        self.eulerData = dict()
        self.eulerData["Y"] = 0.0 
        self.eulerData["P"] = 0.0
        self.eulerData["R"] = 0.0

        # define topic and service names
        self.eulerDataTopic = "/bno055_euler"
        self.buttonDataTopic = "/controller_button_event"
        self.lidarTopic = "/scan"
        self.hapticVoiceSrvName = "/haptic_voice_event"
        
        # define scaling and limit constants
        self.currentScaleFactor = 0.5
        self.incDecAmount = 0.05
        self.deadZonePitchPlus = 6.5
        self.deadZonePitchMinus = -6.5
        self.deadZoneRollPlus = 6.5
        self.deadZoneRollMinus = -6.5
        self.maxAnglePitch = 30
        self.maxAngleRoll = 30
        self.velExecTime = 0.005

        # define constants for obstacle detection
        self.lidarDistanceThresh = 1
        self.maxNumBumps = 10
        self.timeOfLastDisplay = time.time()
        self.minTimeBetweenDisplays = 2 # seconds
        self.collisionVoiceID = 2
        self.collisionHapticID = 52

    def start_node(self):
        rospy.init_node(self.nodeName)
        rospy.loginfo("started " + self.nodeName)

        # create locobot instance
        arm_config = dict(control_mode='torque')
        self.LoCoBotInstance = Robot('locobot', arm_config=arm_config)

        # init subscribers
        rospy.Subscriber(self.eulerDataTopic, bno055_euler_data, self.euler_callback)
        rospy.Subscriber(self.buttonDataTopic, controller_event_data, self.button_callback)
        rospy.Subscriber(self.lidarTopic, LaserScan, self.lidar_callback)

        rospy.spin()

    def euler_callback(self, msg):
        """
        this function extracts the sensor data from the recieved
        message payload and updates the values within the class itself.

        params:
            msg (bno055_euler_data instance): recieved message
        returns:
            None
        """
        # make sure we are using the right controller for
        # the recieved message.
        if(msg.controller_name == self.controller_id):
            self.eulerData["Y"] = msg.yaw
            self.eulerData["P"] = msg.pitch
            self.eulerData["R"] = msg.roll

            self.scale_and_set_vel()
        else:
            pass

    def button_callback(self, msg):
        """
        this function will increment or decrement the scale
        factor for motion based on user button presses.

        params:
            msg (controller_event_data instance): recieved message
        returns:
            None
        """
        # first check to make sure we are using the right controller
        if(msg.controller_name == self.controller_id):
            # increment or decrement scale factor based on event type
            if(msg.event_type == "top_button_event"):
                self.currentScaleFactor += self.incDecAmount
            elif(msg.event_type == "bottom_button_event"):
                self.currentScaleFactor -= self.incDecAmount
        else:
            pass

    def lidar_callback(self, msg):
        """
        This function will check if self.maxNumBumps of returns
        are within self.lidarDistanceThresh of the robot base. If 
        this condition is met, a audio-haptic effect is sent to the controller

        params:
            msg (LaserScan msg): recieved scan message
        return:
            None
        """
        start_angle = float(msg.angle_min)
        end_angle = float(msg.angle_max)
        angle_inc = float(msg.angle_increment)
        points = msg.ranges

        num_consecutive_bumps = 0
        for pt in points:
            # check if current point is within 
            # threshold or not
            if(pt < self.lidarDistanceThresh):
                num_consecutive_bumps += 1
            else:
                num_consecutive_bumps = 0

            # check if we have enough 'bumps' for audio-haptic display
            if(num_consecutive_bumps >= self.maxNumBumps):
                # make sure we've waited awhile before replaying display
                time_diff = time.time() - self.timeOfLastDisplay
                if(time_diff > self.minTimeBetweenDisplays):
                    # make call and reset time
                    self.haptic_voice_srv_call(self.collisionHapticID, self.collisionVoiceID)
                    self.timeOfLastDisplay = time.time()
                else:
                    pass
            else:
                pass

    def haptic_voice_srv_call(self, haptic_id, voice_id):
        """
        this function allows for control of a given controller's
        haptic actuator and TTS engine.

        params:
            haptic_id (int): ID of the haptic effect to play (1->117)
            voice_id (int): ID of the voice effect to play (0->2)

        returns:
            res (bool) the result of the service call
        """
        rospy.wait_for_service(self.hapticVoiceSrvName)

        try:
            # create message
            controller_event_msg = controller_haptic_voice()
            controller_event_msg.header.stamp = rospy.Time.now()
            controller_event_msg.controller_name = self.controller_id
            controller_event_msg.haptic_action_id = haptic_id 
            controller_event_msg.voice_action_id = voice_id

            controller_event = rospy.ServiceProxy(self.hapticVoiceSrvName, haptic_voice_event)
            res = event(controller_event_msg)
            return res.status
        except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s"%e)

    def scale_and_set_vel(self):
        # check if we are in pitch or roll deadzones
        curRoll = self.eulerData["R"]
        curPitch = self.eulerData["P"]
        inPitchDeadzone = curPitch < self.deadZonePitchPlus and curPitch > self.deadZonePitchMinus
        inRollDeadzone = curRoll < self.deadZoneRollPlus and curRoll > self.deadZoneRollMinus

        # see if we are above max angles
        overMaxPitch = abs(curPitch) > self.maxAnglePitch
        overMaxRoll = abs(curRoll) > self.maxAngleRoll

        # if we are not in deadzones, and not greater than max angle
        # try to scale to usable velocity
        unit_fwd_vel = 0
        unit_twist_vel = 0
        if(not inPitchDeadzone and not overMaxPitch):
            isPitchNeg = curPitch < 0
            if(isPitchNeg):
                # get angle as fraction of 'usable' workspace
                totalUsableNegative = self.deadZonePitchMinus - self.maxAnglePitch
                negFraction = (curPitch + self.deadZonePitchMinus)/totalUsableNegative
                unit_fwd_vel = negFraction
            else:
                # get angle as fraction of 'usable' workspace
                totalUsablePositive = self.deadZonePitchPlus + self.maxAnglePitch
                posFraction = (curPitch + self.deadZonePitchPlus)/totalUsablePositive
                unit_fwd_vel = posFraction

        # handle OOB cases
        elif(inPitchDeadzone):
            unit_fwd_vel = 0
        elif(overMaxPitch):
            unit_fwd_vel = 1

        if(not inRollDeadzone and not overMaxRoll):
            isRollNeg = curRoll < 0
            if(isRollNeg):
                # get angle as fraction of 'usable' workspace
                totalUsableNegative = self.deadZoneRollMinus - self.maxAngleRoll
                negFraction = (curRoll + self.deadZoneRollMinus)/totalUsableNegative
                unit_twist_vel = negFraction
            else:
                # get angle as fraction of 'usable' workspace
                totalUsablePositive = self.deadZoneRollPlus + self.maxAngleRoll
                posFraction = (curRoll + self.deadZoneRollPlus)/totalUsablePositive
                unit_twist_vel = posFraction

        # handle OOB cases
        elif(inRollDeadzone):
            unit_twist_vel = 0
        elif(overMaxRoll):
            unit_twist_vel = 1

        # scale velocities
        unit_fwd_vel *= self.currentScaleFactor
        unit_twist_vel *= self.currentScaleFactor

        rospy.loginfo('current velocity (fwd, twist): (%s, %s)' % (unit_fwd_vel, unit_twist_vel))

        # set robot vel
        self.LoCoBotInstance.base.set_vel(fwd_speed=unit_fwd_vel,
                turn_speed=unit_twist_vel,
                exe_time=self.velExecTime)

if(__name__ == "__main__"):
    try:
        controller_id = "right_hand"
        haptic_base_control = hapticBaseControl(controller_id)
        haptic_base_control.start_node()

    except rospy.ROSInterruptException:
        pass


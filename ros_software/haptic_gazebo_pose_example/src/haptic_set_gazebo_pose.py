#!/usr/bin/python

# Author: Trevor Sherrard
# Course: Directed Research
# Since: 07/25/2021
# Description: This script subscribes to the pose constructed by fusing intel realsense
#              and BNO055 sensor data and moves an object in gazebo accordingly.

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import QuaternionStamped

# misc variables
orientationTopic = "/bno055_quat"
gazeboServiceName = "/gazebo/set_model_state"
y_offset = 1 # offset in meters of camera from floor
scale_factor = 10

def poseCallback(msg):
    """
    This function subscribes to the pose of the detected vision target
    and contructs a model state message from this pose. From here, the
    ModelState message is used to make a rosservice call to move an object in 
    gazebo accordingly

    params:
        msg (geometry_msgs/Pose): The vision target pose message
    returns;
        None
    """
    # wait fo gazebo to come up
    rospy.wait_for_service(gazeboServiceName)

    # constuct modelState message from recieved pose
    stateMsg = ModelState()
    stateMsg.model_name = "simpleBox"

    # set position as a static value until we get intel realsense working
    # with new controller.
    stateMsg.pose.position.x = 1
    stateMsg.pose.position.y = 1
    stateMsg.pose.position.z = 1
    stateMsg.pose.orientation.x = msg.quaternion.x
    stateMsg.pose.orientation.y = msg.quaternion.y
    stateMsg.pose.orientation.z = msg.quaternion.z
    stateMsg.pose.orientation.w = msg.quaternion.w

    # attempt to make ROS service call to change gazebo model Pose
    try:
        setModelState = rospy.ServiceProxy(gazeboServiceName, SetModelState)
        response = setModelState(stateMsg)
    except rospy.ServiceException, err:
        rospy.logerr("service call failed: %s" % err)

def main():
    # init node
    rospy.init_node("set_gazebo_box_pose")
    rospy.loginfo("set_gazebo_box_pose initialized")

    # start subscriber
    rospy.Subscriber(orientationTopic, QuaternionStamped, poseCallback)
    rospy.spin()

if(__name__ == "__main__"):
    main()

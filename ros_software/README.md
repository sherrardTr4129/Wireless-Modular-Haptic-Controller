# ROS Software
This sub-directory outlines the ROS software developed to showcase the capabilities of the modular haptic controller. In particular, this sub-directory includes a ROS driver for bi-directional interfacing with the controller network via standard ROS interfaces. It also included a ROS package that interfaces with said ROS driver to demonstrate the control of a robot base using the controller in its hand-held configuration. 

## Inbound and Outbound Data Packet JSON Formats
This sub-section outlines the JSON schema for the inbound and outbound data packets for a given controller on the controller mesh network.

### Outbound data JSON schema
The format of outbound BNO055 data packets can be seen below (if usingEuler is set to true):
```JSON
{
  "packet_type": "outbound_data",
  "controller_id": "right_hand",
  "euler_x": 0,
  "euler_y": 0,
  "euler_z": 0,
  "temp_c": 30
}
```

If usingEuler is set to false, the format of the outbound BNO055 data packet will be:
```JSON
{
  "packet_type": "outbound_data",
  "controller_id": "right_hand",
  "quat_x": 0,
  "quat_y": 0,
  "quat_z": 0,
  "quat_w": 0,
  "temp_c": 30
}
```

### Inbound data JSON schema


## ROS Coordinator Node Serial Driver

### Custom Messages

### Custom Services

## LoCoBot Base Control Example

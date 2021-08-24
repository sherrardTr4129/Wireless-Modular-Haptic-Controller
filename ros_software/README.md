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

Note these are both just templates, and the value of a given field will change.

The other outbound data type from a given controller is event data packets. These are transmitted whenever user input is obtained through the pushbuttons attached to a given controller. The format of this type of data packet can be seen below:
```JSON
{
  "packet_type": "button_event",
  "controller_id": "right_hand",
  "event_type": "top_button_pressed"
}
```

### Inbound data JSON schema
Inbound JSON data packets contain a target controller, and a requested audio or haptic event to display to the user. These data packets take the form of the following JSON schema:

```JSON
{
  "controller_name": "right_hand",
  "voice_action_id": 1,
  "haptic_action_id": 53
}
```

More on the meanings of the values for the voice\_action\_id and haptic\_action\_id fields in the upcomming sections. The JSON seen above will be created within the ROS driver itself, so don't worry too much about this.

## ROS Coordinator Node Serial Driver
The haptic\_orientation\_capture\_driver ROS node serves as a bridge between the controller mesh network and a given ROS software architecture. This driver will manage the serial interfacing between a base-station coordinator XBee and the host computer. This driver will also publish recieved BNO055 data streams and button press events. This driver also exposes a ROS service that can be called by external ROS nodes to send audio-haptic event requests to a given controller on the mesh network. 

### Custom Messages
The packages outlined here make use of a few custom ROS messages to communicate between the ROS infrustructure and the controller mesh network. Please have a look at the table below for an outline of these messages.

| Message Name                | Message Components                                                                                                                                                                                                                                                                                                                                                                                             |
|-----------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| bno055_euler_data.msg       | **Description** - BNO055 data received from mesh network is published to ROS infrastructure through this message (euler format) <br><br>**Message Content:**<br>string controller\_name - The controller the BNO055 data came from<br>Header header - header containing a timestamp<br>float64 roll - roll data (in degrees)<br>float64 pitch - pitch data (in degrees)<br>float64 yaw - yaw data (in degrees) |
| bno055_quat_data.msg        | **Description** - BNO055 data received from mesh network is published to ROS infrastructure through this message (quaternion format)<br><br>**Message Content:**<br>string controller\_name - The controller the BNO055 data came from<br>geometry_msgs/QuaternionStamped quat - orientation data as quaternion                                                                                                |
| controller_event_data.msg   | **Description** - button press event data received from mesh network is published to ROS infrastructure through this message<br><br>**Message Content:**<br>string controller\_name - The controller the button event data came from<br>Header header - header containing a timestamp<br>string event_type - string representation of which button was pressed on the controller                               |
| controller_haptic_voice.msg | **Description** - audio-haptic requests can be sent to a given controller on the mesh network via the ROS service described below. <br>This message is used to make said request.<br><br>**Message Content:**<br>string controller\_name - The controller the request is bound for<br>int16 haptic_action_id - selected haptic display ID<br>int16 voice_action_id - selected TTS display ID                   |

### Custom Service
The haptic\_orientation\_capture\_driver ROS node hosts a service that allows users to request an audio-haptic event to be performed by a given controller. This service call requires a haptic\_voice\_event message instance containing the voice\_action\_id and haptic\_action\_id for a given controller with name controller\_name. For the pre-rendered TTS events, please see firmware to determine mapping between voice\_action\_id and a given TTS event. For the haptic diplays, haptic\_action\_id should be chosen from the table below based on the desired waveform.

## LoCoBot Base Control Example

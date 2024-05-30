# BC
## Flavian Annus - Controlling a Drone Via Gestures

This project enables controlling a simulated drone via gestures using Leap Motion, PX4, and a ROS2 environment.

## Installation Guides

- [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/index.html)
- [PX4 Development Environment Setup](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#simulation-and-nuttx-pixhawk-targets)
- [ROS2 Gazebo Bridge](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge) (highly advised for camera output from the drone)
- [Leap Motion SDK for Linux](https://docs.ultraleap.com/linux/?_gl=1*1otp0h2*_ga*NDE1ODUyMDIyLjE3MDAxNDc2NDE.*_ga_5G8B19JLWG*MTcwMDQ3NTAyNS40LjEuMTcwMDQ3NTAzMi41My4wLjA)

## Project Overview

This project consists of six nodes designed to facilitate the control of a simulated drone using gestures:

1. **Gesture Interface Node**: Reads data from the Leap Motion controller.
2. **Gesture Node**: Processes and calibrates hand data, then translates it into gestures.
3. **Control Node**: Converts gestures into commands for the drone.
4. **Communication Hub**: Sends commands from the control node to the simulation.
5. **RPY Node**: Pulls Roll, Pitch, and Yaw data from the gesture interface node.
6. **Virtual Hand Node**: Visualizes data from the Leap Motion controller in RViz2.

### Node Descriptions

#### Gesture Interface Node
This node is responsible for interfacing with the Leap Motion controller and obtaining raw hand movement data.

#### Gesture Node
This node processes the raw data, calibrates hand movements, and recognizes specific gestures.

#### Control Node
This node translates recognized gestures into specific commands for controlling the drone.

#### Communication Hub
Acts as a bridge between the control node and the simulation environment, sending commands to the simulated drone.

#### RPY Node
Extracts Roll, Pitch, and Yaw data from the Gesture Interface Node for additional processing and control refinement.

#### Virtual Hand Node
Visualizes hand movements and gestures in RViz2, aiding in debugging and development.

## Gesture Specifications

The following sections describe the gestures used to control the drone. Each gesture will be illustrated with an image.

### Gesture 1: [Description]
![Gesture FIVE]([text](../Downloads/five))

### Gesture 2: [Description]
![Gesture FIST]([text](../Downloads/fist))

### Gesture 3: [Description]
![Gesture THUMB_L]([text](../Downloads/thl))

### Gesture 4: [Description]
![Gesture THUMB_R]([text](../Downloads/th))

### Gesture 5: [Description]
![Gesture ONE]([text](../Downloads/one))

### Gesture 6: [Description]
![Gesture TWO]([text](../Downloads/two))

## Getting Started

### Prerequisites

Ensure you have the following installed and configured:
- ROS2 Humble
- PX4 Autopilot
- Gazebo with ROS2 bridge
- Leap Motion SDK

### Installation

Follow the installation guides provided above to set up your environment.

### Running the Nodes

1. Launch the Gesture Interface Node to start reading data from the Leap Motion controller.
2. Start the Gesture Node to begin processing and recognizing gestures.
3. Run the Control Node to convert gestures into drone commands.
4. Initialize the Communication Hub to relay commands to the simulation.
5. Start the RPY Node to extract orientation data.
6. Launch the Virtual Hand Node to visualize the hand movements in RViz2.

### Usage

Once all nodes are running, use the specified gestures to control the simulated drone. Refer to the gesture specifications for details on each control gesture.

## Troubleshooting

If you encounter any issues, ensure all dependencies are correctly installed and that each node is properly configured.

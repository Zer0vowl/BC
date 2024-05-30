# Leap Gesture Interface for Drone Control

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

## Installation

1. **Set up a ROS 2 workspace** (skip if you already have one):
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```

2. **Clone this repository**:
    ```bash
    cd src/
    git clone https://github.com/Zer0vowl/BC.git
    ```

3. **Install dependencies**:
    Navigate back to your ROS 2 workspace root and run:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

4. **Build the package**:
    ```bash
    colcon build
    ```

5. **Source the environment**:
    ```bash
    . install/setup.bash
    ```

## Gesture Specifications

The following sections describe the gestures used to control the drone. Each gesture will be illustrated with an image.

### Gesture FIVE:
![Gesture FIVE](src/bc_pkg/config/gestures/five)

### Gesture FIST:
![Gesture FIST](src/bc_pkg/config/gestures/fist)

### Gesture ONE:
![Gesture ONE](src/bc_pkg/config/gestures/one)

### Gesture TWO:
![Gesture TWO](src/bc_pkg/config/gestures/two)

### Gesture THUMB_L:
![Gesture THUMB_L](src/bc_pkg/config/gestures/thl)

### Gesture THUMB_R:
![Gesture THUMB_R](src/bc_pkg/config/gestures/th)

## Usage

### Running the Nodes

1. Launch the Gesture Interface Node to start reading data from the Leap Motion controller.
2. Start the Gesture Node to begin processing and recognizing gestures.
3. Run the Control Node to convert gestures into drone commands.
4. Initialize the Communication Hub to relay commands to the simulation.
5. Start the RPY Node to extract orientation data.
6. Launch the Virtual Hand Node to visualize the hand movements in RViz2.


1. **Start the drone simulation**
    ```bash
    cd ~/PX4-Autopilot && make px4_sitl gz_x500_mono_cam
    ```

2. **In a new terminal, start the MicroXRCE Agent**
    ```bash
    MicroXRCEAgent udp4 -p 8888
    ```

3. **To run the gz bridge for the camera output**
    ```bash
    ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image
    ```

4. **In your workspace to start the CommunicationHub node**
    ```bash
    ros2 run bc_pkg connect
    ```
5. **In your workspace to start the Control node**
    ```bash
    ros2 run bc_pkg control
    ```
6. **In your workspace to start the GestureInterface node**
    ```bash
    ros2 run bc_pkg carth
    ```

7. **In your workspace to start the RPY node**
    ```bash
    ros2 run bc_pkg rpy
    ```

8. **In your workspace to start the Gesture node**
    ```bash
    ros2 run bc_pkg gest
    ```

9. **In your workspace to start the VirtualHand node**
    ```bash
    ros2 run bc_pkg hand
    ```

9. **In order to visualize the data captured by the leap motion sensor**
    ```bash
    rviz -d ~/[your_workspace_name]/src/bc_pkg/config/virtual_hand.rviz
    ```

9. **In order to visualize the camera outputt**
    ```bash
    rviz -d ~/[your_workspace_name]/src/bc_pkg/config/cam.rviz
    ```

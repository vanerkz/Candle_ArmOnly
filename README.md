# Candle_ArmOnly

Here’s a more structured format for your README file using common conventions. It’s organized with headers, subheaders, bullet points, and code blocks for better clarity.

````markdown
# Feetech Arm Simulation with ROS Noetic and MoveIt

This repository provides instructions for setting up and running a **Feetech robotic arm simulation** in **Gazebo** and controlling it with **RViz** using **ROS Noetic** and **MoveIt**. The robot utilizes **Feetech RS485 Modbus-RTU servos**.

## Table of Contents

- [Requirements](#requirements)
- [Installation](#installation)
  - [Install ROS Noetic](#install-ros-noetic)
  - [Install MoveIt](#install-moveit)
  - [Clone the Repository](#clone-the-repository)
  - [Build the Workspace](#build-the-workspace)
  - [Run the Simulation](#run-the-simulation)
- [Simulation Environment](#simulation-environment)
  - [RViz](#rviz)
  - [Gazebo](#gazebo)
- [Robot Description](#robot-description)
  - [URDF Files](#urdf-files)
  - [Mesh Files](#mesh-files)
  - [Servos Used](#servos-used)
- [Adjusting Robot Height](#adjusting-robot-height)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Requirements

- **Operating System**: Ubuntu 20.04
- **ROS Version**: Noetic
- **ROS Packages**:
  - [ROS Noetic Installation](https://wiki.ros.org/noetic/Installation/Ubuntu)
  - [MoveIt](https://moveit.github.io/moveit_tutorials/doc/getting_started/getting_started.html)

## Installation

### Install ROS Noetic
Follow the official guide to install ROS Noetic on Ubuntu 20.04:
[ROS Noetic Installation](https://wiki.ros.org/noetic/Installation/Ubuntu)

### Install MoveIt
Install MoveIt for ROS Noetic by following the [MoveIt Getting Started Tutorial](https://moveit.github.io/moveit_tutorials/doc/getting_started/getting_started.html).

### Clone the Repository

```bash
# Create the catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone this repository
git clone <repository_url>
````

### Build the Workspace

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

### Run the Simulation

To launch the robot arm simulation, use the following command:

```bash
roslaunch arm_only demo_gazebo.launch
```

This will launch both **RViz** and **Gazebo**, allowing you to control the robot from **RViz** and see the movement in **Gazebo**.

## Simulation Environment

### RViz

* **RViz** is used for controlling and visualizing the robot in 3D.
* You can move joints and visualize the robot's movements in real-time.

### Gazebo

* **Gazebo** provides a physics-based simulation of the robot in a realistic 3D environment.
* You can observe the actual motions of the robot in the simulated world.

## Robot Description

### URDF Files

The **robot's URDF** (Unified Robot Description Format) file is located at:

```
../catkin_ws/src/urdf_tutorial/urdf/feetecharm.urdf.xacro
```

### Mesh Files

The **mesh files** (STL files) for the robot are located at:

```
/home/robot/catkin_ws/src/urdf_tutorial/meshes
```

### Servos Used

The robot uses **Feetech RS485 Modbus-RTU servos**. The specific models are:

* 2x 24V 45kg RS485 Modbus-RTU servos
* 2x 12V 85kg RS485 Modbus-RTU servos
* 2x 12V 120kg RS485 Modbus-RTU servos
* 1x 12V 180kg RS485 serial bus servo

You can find more information about these servos at [Feetech RS485 Series Servos](https://www.feetechrc.com/products/sms_rs485_series%20servo-page-1).

## Adjusting Robot Height

To adjust the robot's height in the **Gazebo** simulation, edit the following line in the `gazebo_feetech.urdf` file:

```xml
<origin rpy="0 0 0" xyz="0 0 1" />
```

This is located at:

```
../catkin_ws/src/arm_only/config/gazebo_feetech.urdf
```

Change the `z` value to set the desired height for the robot.

Example to set the robot's height to 1.5 meters:

```xml
<joint name="world_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 1.5" />
    <parent link="world" />
    <child link="base_link" />
</joint>
```

## Troubleshooting

* **Simulation issues**: Ensure all dependencies are correctly installed by running `rosdep install --from-paths src`.
* **RViz or Gazebo not launching**: Check ROS logs for error messages.
* **Height adjustment not working**: Double-check the `xyz` values in the `gazebo_feetech.urdf` file.

## License

This project is licensed under the **MIT License**. See the [LICENSE](LICENSE) file for details.

```

---

### Key Sections Explained:

- **Table of Contents**: Helps users navigate through the different sections of the README.
- **Installation**: Clear step-by-step instructions for setting up ROS, MoveIt, and cloning/building the repository.
- **Simulation Environment**: Describes the tools (RViz and Gazebo) and their usage.
- **Adjusting Robot Height**: Guides users to modify the robot's height in the simulation environment.
- **Troubleshooting**: Offers some common solutions to potential issues.

This format is easy to follow, especially for new users setting up a simulation or dealing with common setup problems!
```

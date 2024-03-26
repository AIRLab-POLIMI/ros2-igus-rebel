# Soft Gripper controller: ROS2-Control Hardware Interface using GPIO interface

## Description

The pneumatic pump actuating the soft gripper is controlled by an Arduino, running the code inside the `src/pump_control` directory.
In order to actuate the soft gripper it is sufficient to send the correct commands as string via serial interface, using the UART protocol.
The commands will be received by the Arduino, which will then actuate the pneumatic pump accordingly. 

The soft gripper can be operated via a ros2 service call. The service server listens for string commands that are then sent via serial 
interface to the Arduino controller. The service call can be used to open, close or turn off the pneumatic pump actuating the gripper.

## Usage

To launch the service via CLI, you can type in a terminal the following command, where `<command>` can be either `release`, `grip` or `off`:

```bash
ros2 service call /gripper_actuate igus_rebel_gripper_controller/srv/GripperActuation "{command: '<command>'}"
```

It is also required that the main controllers are launched before using the gripper service. The main controllers can be launched 
using the following command:

```bash
ros2 launch igus_rebel_moveit_config demo.launch.py mount:=mount_v2 end_effector:=soft_gripper load_base:=true camera:=realsense
```

### Configuration

The default port is set to `/dev/ttyACM0`, but it can be changed to any other port where the Arduino is connected.
The port at which the Arduino is connected can be configured inside the `soft_gripper.control.xacro` file, in the folder
`igus_rebel_description_ros2/src/urdf/end_effectors/`. 
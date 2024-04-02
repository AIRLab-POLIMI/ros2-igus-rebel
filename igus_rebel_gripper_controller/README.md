# Soft Gripper controller: ROS2-Control Hardware Interface using GPIO interface

## Description

The pneumatic pump actuating the soft gripper is controlled by an Arduino, running the code inside the `src/pump_control` directory.
In order to actuate the soft gripper it is sufficient to send the correct commands as string via serial interface, using the UART protocol.
The commands will be received by the Arduino, which will then actuate the pneumatic pump accordingly. 

The soft gripper can be operated via a ros2 service call. The service server listens for string commands that are then sent via serial 
interface to the Arduino controller. The service call can be used to open, close or turn off the pneumatic pump actuating the gripper.

## Establish serial communication

Before using the service call, it is necessary to establish serial communication between the Arduino and the computer.
There are 2 ways to do this:
1. Using the Arduino IDE: launch the Arduino IDE and open and close the serial monitor
2. Use a sequence of commands in the terminal to create a serial connection between the Arduino and the computer.

The method 2 requires to run the following commands in the terminal:
```bash
stty -F /dev/ttyACM0 raw ispeed 115200 ospeed 115200 cs8 -ignpar -cstopb -echo
cat < /dev/ttyACM0 > /dev/null &
```
then test the connection with the following command:
```bash
echo -e "grip" > /dev/ttyACM0
```

Substitute `/dev/ttyACM0` with the port at which the Arduino is connected. The port can be found by running the following command:
```bash
ls -l /dev/tty*
```

To make it more convenient to establish the serial connection, a series of __bash aliases__ can be created to run the above commands
as soon as the bash command line is launched. To do so, add the following lines to the `.bashrc` file:

```bash
# enable serial communication on port /ttyACM0 for arduino
alias enable_arduino='stty -F /dev/ttyACM0 raw ispeed 115200 ospeed 115200 cs8 -ignpar -cstopb -echo & cat < /dev/ttyACM0 > /dev/null &'
# pump control commands via bash: quick commands
alias pump_grip='echo -e "grip" > /dev/ttyACM0'
alias pump_off='echo -e "off" > /dev/ttyACM0'
alias pump_release='echo -e "release" > /dev/ttyACM0'
```

## ROS2-Control Usage

To launch the service via CLI, you can type in a terminal the following command, where `<command>` can be either `release`, `grip` or `off`:

```bash
ros2 service call /gripper_actuate igus_rebel_gripper_controller/srv/GripperActuation "{command: '<command>'}"
```

It is also required that the main controllers are launched before using the gripper service. The main controllers can be launched 
using the following command:

```bash
ros2 launch igus_rebel_moveit_config demo.launch.py mount:=mount_v2 end_effector:=soft_gripper load_base:=true camera:=realsense
```

### Port Configuration

The default port is set to `/dev/ttyACM0`, but it can be changed to any other port where the Arduino is connected.
The port at which the Arduino is connected can be configured inside the `soft_gripper.control.xacro` file, in the folder
`igus_rebel_description_ros2/src/urdf/end_effectors/`. 
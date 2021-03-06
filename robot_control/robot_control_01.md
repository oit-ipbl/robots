# Robot control (1)

[README](../README.md)

---

## Objectives

This page explains how to control the robot with command, keyboard and mouse.

## Prerequisite

You have to finish [ROS basics](../basics/basics_01.md).

## Launch simulator

Open a linux terminal emulator. See [Use terminal Emulator in the ROS Container](https://github.com/oit-ipbl/portal/blob/main/setup/dockerros.md#use-terminal-emulator-in-the-ros-container), and input the following command.

```shell
$ roslaunch oit_stage_ros navigation.launch
```

Open another terminal emulator and type `rostopic list` commmand.

Imporant topics are as follows,

```shell
$ rostopic list
...
/base_scan # Virtual laser range finder's data
...
/cmd_vel # Velocity command to the robot
...
/image   # Picture grabbed by the virtual camera 
...
/odom    # Robot's position with some errors
...
/robot_face_image # Robot's facial image
...
```

## :o:Exercise(Control the robot via command line)

`rostopic pub` command is used to publish the `cmd_vel` topic.

The data type of `cmd_vel` is [`geometry_msgs/Twist Message`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html), which is complex structure.

We strongly recommend `Tab` completion to input the following command.

```shell
$ rostopic pub /cmd_vel geometry_msgs/Twist "linear:
 x: 0.4
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 0.0"  -r 10
```

Example `Tab` completion process is here,

```shell
$ rostopic pub /cmd_vel # Press Tab key about 2~3 times
```

Then, almost of the command will appear.

By the command, the robot will go forward with 0.4m/sec and stops because of a wall.
Press `Ctrl+C` to terminate the command.

Try following command to make the robot turns. `angular: z` is an angular velocity described with radian.

```shell
$ rostopic pub /cmd_vel geometry_msgs/Twist "linear:
 x: 0.0
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 0.52"  -r 10
```

Terminate by `Ctrl+C`.

## :o:Exercise(Control the robot with key board)

Re-start the simulator, and open another terminal emulator for the following command.

```shell
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

On the terminal, use the following 4 keys to control the robot.

- `i`: forward, `j`: left turn `l`: right turn `,`: back

Terminate the `teleop_twist_keyboard.py` by `Ctrl+C`.

## :o: Exercise(Control the robot with mouse)

Type the following command. You can see one GUI window named 'Mouse Teleop'.

```shell
$ rosrun  mouse_teleop mouse_teleop.py mouse_vel:=cmd_vel
```

You can send velocities to the robot by dragging your mouse on the 'Mouse Teleop' window.

![2020-02-07_13-14-59.png](./2020-02-07_13-14-59.png)

- **Important** You have to press close button on the GUI 'Mouse Teleop' windows to terminalte `mouse_teleop.py`.


---

[README](../README.md)

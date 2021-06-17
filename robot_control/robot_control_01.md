# Robot control (1)

[README](../README.md)

---

In this lecture, we will control the robot with command and tools.

## Launch simulator

```shell
$ roslaunch oit_stage_ros navigation.launch
```

Open another terminal and type `rostopic list` commmand.

Imporant topics are as follows,

```shell
$ rostopic list
...
/base_scan # Virtual laser range finder's data
...
/cmd_vel # Velocity command to robot
...
/image   # Picture grabbed by virtual camera 
...
/odom    # Robot's position with some errors
...
/robot_face_image # Robot's facial image
...
```

## Control the robot via command line

`rostopic pub` command is used for publish the `cmd_vel` topic.

The data type of `cmd_vel` is [`geometry_msgs/Twist Message`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html), which is complex structure.

We strongly recommend `tab` completion to input the following command.

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
$ rostopic pub /cmd_vel # Press tab key about 2~3 times
```

Then, almost of the command will appear.

By the command, the robot will go forward with 0.4m/sec and stops by a wall.
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

Terminate with `Ctrl+C`.

## Control the robot with key board

Reboot the simulator, and open another terminal fot the following command

```shell
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

On the terminal, you use the 4 keys to control the robot.

- `i`: forward, `j`: left turn `l`: right turn `,`: back

Terminate by `Ctrl+C`.

## Control the robot with mouse

Type the following command. You can see one GUI window named 'Mouse Teleop'.

```shell
$ rosrun  mouse_teleop mouse_teleop.py mouse_vel:=cmd_vel
```

You can send velocities to the robot by draggin your mouse on the 'Mouse Teleop' window.

![2020-02-07_13-14-59.png](./2020-02-07_13-14-59.png)

- **Important** You have to press close button on the GUI 'Mouse Teleop' windows to terminalte mosue teleop.

## ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+)Checkpoint(robot control)

- It's OK, you can control the robot with command, keyboard and mouse.

---

[README](../README.md)

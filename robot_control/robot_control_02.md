# Robot control (2)

[README](../README.md)

---

## Objectives

This page explains how to make a program which sends velocities to the robot via `/cmd_vel` topic.

## Prerequisite

You have to finish [Robot control (1)](robot_control/robot_control_01.md).

## Practice [go forward and turn]

Make a python file inside of the `oit_pbl_ros_samples` package.  
Open a linux terminal emulator. See [Use terminal Emulator in the ROS Container](https://github.com/oit-ipbl/portal/blob/main/setup/dockerros.md#use-terminal-emulator-in-the-ros-container), and input the following commands.

```shell
$ roscd oit_pbl_ros_samples/scripts
$ pwd
/home/ubuntu/catkin_ws/src/oit_pbl_ros_samples/scripts
$ touch robot_control.py
$ chmod u+x robot_control.py
```

Edit the `robot_control.py`.

- Open `~/catkin_ws/src/oit_pbl_ros_samples/` by Visual Studio Code editor, and edit `robot_control.py`.

Type the following template. It's OK copy and paste.

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import rospy
from geometry_msgs.msg import Twist


class RobotControlNode(object):
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def process(self):
        rate = rospy.Rate(10)  # Keep loop with 10hz
        cmd = Twist()
        cmd.linear.x = 0.4  # linear velocity
        start = rospy.Time.now()
        while rospy.Time.now().to_sec() - start.to_sec() < 3:  # 3 seconds
            self.pub_cmd_vel.publish(cmd)
            rate.sleep()  # Keep loop with 10hz
        cmd.linear.x = 0.0
        cmd.angular.z = math.radians(30)  # angular velocity
        start = rospy.Time.now()
        while rospy.Time.now().to_sec() - start.to_sec() < 3:  # 3 seconds
            self.pub_cmd_vel.publish(cmd)
            rate.sleep()  # Keep loop with 10hz


def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.

    node = RobotControlNode()
    rospy.loginfo("%s:Started", rospy.get_name())

    node.process()
    rospy.loginfo("%s:Exiting", rospy.get_name())


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr("%s:%s", rospy.get_name(), str(e))
        exit(1)
```

### Run

At first, launch the simulator.

```shell
$ roslaunch oit_stage_ros navigation.launch
```

After a while run the `robot_control.py`.

- Carefully check the screen of `Stage` and `Rviz`.
- The robot go forward with 0.4m/sec for 3 seconds. And turn with 30 degrees/sec for 3 seconds.

```shell
$ rosrun oit_pbl_ros_samples robot_control.py
[INFO] [1623920868.296397, 8.500000]: /robot_control:Started
[INFO] [1623920874.284430, 14.500000]: /robot_control:Exiting
```

## Exercise (robot control 2-1)

- Modify `robot_control.py` to make the following robot's behavior'.
- Go forward (some seconds) -> Turn (about 360 degrees) -> Go forward (some seconds).

## Exercise (robot control 2-2)

- Modify `robot_control.py` to make a square movement of the robot
  - Go forward (some seconds) -> 90 degrees turn -> Go forward (some seconds) ...
- Implement clockwise and counter clockwise square movement.

## ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+)Checkpoint(robot control 2)

- It's OK, you can finish the exercise 1 and 2.

## Question (robot control 2)

- Modify `robot_control.py` to assing values to both `cmd.linear.x` and `cmd.angular.z`, and check the robot's movement.

---

[README](../README.md)

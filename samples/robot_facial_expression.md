# Robot facial expression

[README](../README.md)

---

## Objectives

This page explains how to make a program which can change the robot face.

## Change the robot face with command

At first, launch the simulator. Open a linux terminal emulator. See [Use terminal Emulator in the ROS Container](https://github.com/oit-ipbl/portal/blob/main/setup/dockerros.md#use-terminal-emulator-in-the-ros-container), and input the following command.

```shell
$ roslaunch oit_stage_ros navigation.launch
```

- The command launch `~catkin_ws/src/oit_stage_ros/scripts/face_image_publisher.py`, which publishes face image.
  - You can see the robot's face on the `RViz`.
- Open another terminal and type `rostopic pub /robot_face_type std_msgs/String "data: 'happy'" -1`, that changes robot's face.
  - You can use `'sad'` or `'normal'` alternatively.

![2021-04-29_090555.png](./2021-04-29_090555.png)

## Let's think

```shell
rostopic pub /robot_face_type std_msgs/String "data: 'happy'" -1
```

This command publishes the String data `happy` as the topic `robot_face_type`. We can also send another face type string like as `'sad'` and `'normal'`.  
We have learned publisher of String data at the [ROS basics](../basics/basics_01.md), `talker.py`.  

- **So you can make a python program which can change robot's face!**

Before you read following texts, let's try to make a program from scratch.

## Practice (change robot face with python)

Make a python file inside of the `oit_pbl_ros_samples` package.

```shell
$ roscd oit_pbl_ros_samples/scripts
$ pwd
/home/ubuntu/catkin_ws/src/oit_pbl_ros_samples/scripts
$ touch face.py
$ chmod u+x face.py
$ cd ..
```

Open `~/catkin_ws/src/oit_pbl_ros_samples/` by Visual Studio Code editor, and edit `face.py`. See [Developing inside the ROS container with VSCode](https://github.com/oit-ipbl/portal/blob/main/setup/remote_with_vscode.md).

Type the following template. It's OK copy and paste.

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from std_msgs.msg import String


class Face(object):
    def __init__(self):
        self.pub_face = rospy.Publisher(
            '/robot_face_type', String, queue_size=10)

    def process(self):
        types = ["happy", "sad", "normal"]
        for i in range(0, 9):
            self.pub_face.publish(types[i % 3])
            rospy.sleep(3)


def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.

    node = Face()
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

After a while run the `face.py`.

- Carefully check the screen of `Rviz`.
- The robot's face changes every three seconds.

```shell
$ rosrun oit_pbl_ros_samples face.py
[INFO] [1623925455.117405, 2082.000000]: /face:Started
[INFO] [1623925482.101090, 2109.000000]: /face:Exiting
```

## :o:Exercise (face 1)

- Add change face functionality into the `navigation.py`.
  - For example, when robot have reached a goal, the robot express `happy` face for 5 seconds.
  - After that, reset to `normal` face and go to the next goal.

## :o:Challenge (replace face images)

- Edit or replace robot's face image files.

```shell
$ roscd oit_stage_ros/images/faces
$ ls
happy.png  normal.png  sad.png # The face images. Edit them, or replace to your own robot's face image. Do not change the file names.
```

- 640x480 `png` image can be used for this. However, other image size may be acceptable. Please try.
  - Do not change the file names.
- See [Direct access directory and files inside the ROS container](https://github.com/oit-ipbl/portal/blob/main/setup/dockerros.md#direct-access-directory-and-files-inside-the-ros-container) to edit the images from Windows.

## :o:Challenge (add face reaction)

- Add other facial expressions than `happy`, `normal`, `sad`. e.g. `surprise`.

```shell
$ roscd oit_stage_ros/scripts
$ ls 
face_image_publisher.py # This is publisher program of face images.
```

- Modify `face_image_publisher.py` to respond to the following command and show the facial expression of `surprise`.

```shell
rostopic pub /robot_face_type std_msgs/String "data: 'surprise'" -1
```

---

[README](../README.md)

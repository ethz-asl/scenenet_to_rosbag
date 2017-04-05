# Interfacing SceneNet and ROS
Tools for publishing the SceneNet datasets via ROS messages.

## How to use these tools
1. Clone the [pySceneNetRGBD](https://github.com/jmccormac/pySceneNetRGBD) repository in the `src` folder of your catkin workspace.
2. Place the `CMakeLists.txt` and the `package.xml` files from this repository, along with any script you want to use, in the `pySceneNetRGBD` main folder.
3. After following the instructions from the main `pySceneNetRGBD` repository on how to install _protobuf_, download the dataset and generate the _protobuf_ description, you can run the scripts in the `scenenet_ros_tools` package as ROS nodse as follows:

    ```bash
    $ rosrun scenenet_ros_tools some_script.py
    ```
    To run the `pointcloud_publisher.py` script, for example:
    ```bash
    $ rosrun scenenet_ros_tools pointcloud_publisher.py
    ```

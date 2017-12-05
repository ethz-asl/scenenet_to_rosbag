# Interfacing SceneNet and ROS
Tools for publishing the SceneNet datasets via ROS messages.

## How to use these tools
1. Clone the [pySceneNetRGBD](https://github.com/jmccormac/pySceneNetRGBD) repository in the `src` folder of your catkin workspace.

    ```bash
    cd <catkin_ws>/src
    git clone git@github.com:jmccormac/pySceneNetRGBD.git
    ```
2. Place the `CMakeLists.txt` and the `package.xml` files from this repository, along with any script you want to use, in the `pySceneNetRGBD` main folder.

    ```bash
    git clone git@github.com:ethz-asl/scenenet_ros_tools.git /tmp/scenenet_ros_tools
    cp /tmp/scenenet_ros_tools/package.xml /tmp/scenenet_ros_tools/CMakeLists.txt /tmp/scenenet_ros_tools/pointcloud_publisher.py pySceneNetRGBD/
    ```

3. After following the instructions from the [pySceneNetRGBD](https://github.com/jmccormac/pySceneNetRGBD) repository on how to install _protobuf_, download the dataset and generate the _protobuf_ description, you can run the scripts in the `scenenet_ros_tools` package as ROS nodes by running the following command from the `pySceneNetRGBD` folder:

    ```bash
    pip3 install protobuf
    wget http://www.doc.ic.ac.uk/~ahanda/scenenet_rgbd_val.pb <download_location>/scenenet_rgbd_val.pb
    wget http://www.doc.ic.ac.uk/~ahanda/SceneNetRGBD-val.tar.gz <download_location>/SceneNetRGBD-val.tar.gz
    tar -xvzf <download_location>/SceneNetRGBD-val.tar.gz <download_location>
    ln -s <download_location>/scenenet_rgbd_val.pb data/scenenet_rgbd_val.pb  # or cp
    ln -s <download_location>/val data/val  # or cp -R
    make
    ```

    ```bash
    $ rosrun scenenet_ros_tools some_script.py
    ```

    To run the `pointcloud_publisher.py` script, for example:

    ```bash
    rosrun scenenet_ros_tools pointcloud_publisher.py
    ```

# Interfacing SceneNet and ROS
Tools for working with the [SceneNet RGB-D](https://robotvault.bitbucket.io/scenenet-rgbd.html) dataset and converting its trajectories to a ROS bag.

## How to use these tools
1. Clone this and the [pySceneNetRGBD](https://github.com/jmccormac/pySceneNetRGBD) repositories to the `src` folder of your catkin workspace, build your workspace and source it.

    ```bash
    cd <catkin_ws>/src
    git clone git@github.com:ethz-asl/scenenet_ros_tools.git
    git clone git@github.com:jmccormac/pySceneNetRGBD.git
    catkin build
    source <catkin_ws>/devel/setup.bash
    ```

2. Download the SceneNet training (263GB split into 17 tarballs of 16GB each) and/or validation set (15GB) with the respective protobuf files to the `data` directory of the `pySceneNetRGBD` folder, then run `make` in the root `pySceneNetRGBD` folder to generate the protobuf description.

    ```bash
    cd pySceneNetRGBD
    mkdir data && cd data

    # Training set
    wget http://www.doc.ic.ac.uk/~ahanda/train_protobufs.tar.gz train_protobufs.tar.gz
    tar -xvzf train_protobufs.tar.gz --strip=1
    wget http://www.doc.ic.ac.uk/~ahanda/train_split/train_0.tar.gz train_0.tar.gz
    tar -xvzf train_0.tar.gz
    wget http://www.doc.ic.ac.uk/~ahanda/train_split/train_1.tar.gz train_1.tar.gz
    tar -xvzf train_1.tar.gz
    ...

    # Validation set
    wget http://www.doc.ic.ac.uk/~ahanda/scenenet_rgbd_val.pb scenenet_rgbd_val.pb
    wget http://www.doc.ic.ac.uk/~ahanda/SceneNetRGBD-val.tar.gz SceneNetRGBD-val.tar.gz
    tar -xvzf SceneNetRGBD-val.tar.gz

    cd .. && make
    ```

3. Make the Python script executable and run it as a ROS node to convert data from a SceneNet trajectory to a rosbag. The rosbag will contain a sequence of RGB and depth images, ground truth 2D instance label images, and relative transforms. Optionally, it can contain colorized ground truth 2D instance label images, colored pointclouds of the scene, and colored pointclouds of ground truth instance segments.

    ```bash
    cd ../scenenet_ros_tools && chmod +x nodes/scenenet_to_rosbag.py
    rosrun scenenet_ros_tools scenenet_to_rosbag.py --scenenet-path PATH/TO/pySceneNetRGBD --dataset-type DATASET_TYPE --trajectory INDEX  [--train-set-split N] [--limit NUM] [--output-bag NAME]
    ```
    Example for a trajectory from the training set:
    ```bash
    rosrun scenenet_ros_tools  scenenet_to_rosbag.py --scenenet-path ../pySceneNetRGBD/ --dataset-type train --train-set-split 0 --trajectory 1  --output-bag scenenet_train_0_traj_1.bag
    ```
    Example for a trajectory from the validation set:
    ```bash
    rosrun scenenet_ros_tools  scenenet_to_rosbag.py --scenenet-path ../pySceneNetRGBD/ --dataset-type val --trajectory 1 --output-bag scenenet_val_traj_1.bag
    ```
    The output bag contains the following topics:
    ```bash
    # RGB and depth images
    /camera/rgb/camera_info         : sensor_msgs/CameraInfo
    /camera/rgb/image_raw           : sensor_msgs/Image
    /camera/depth/camera_info       : sensor_msgs/CameraInfo
    /camera/depth/image_raw         : sensor_msgs/Image        

    # Ground truth 2D instance segmentation image
    /camera/instances/image_raw     : sensor_msgs/Image

    # Ground truth colorized 2D instance segmentation image [Disabled by default]
    /camera/instances/image_rgb     : sensor_msgs/Image

    # Colored pointclouds of ground truth instance segments [Disabled by default]
    /scenenet_node/object_segment   : sensor_msgs/PointCloud2

    # Colored pointcloud of the scene                       [Disabled by default]
    /scenenet_node/scene            : sensor_msgs/PointCloud2

    # Transform from /scenenet_camera_frame to /world
    /tf                             : tf/tfMessage
    ```

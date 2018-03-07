# Interfacing SceneNet and ROS
Tools for converting the SceneNet datasets to ROS messages in a rosbag.

## How to use these tools
1. Clone this and the [pySceneNetRGBD](https://github.com/jmccormac/pySceneNetRGBD) repositories in the `src` folder of your catkin workspace.

    ```bash
    cd <catkin_ws>/src
    git clone git@github.com:ethz-asl/scenenet_ros_tools.git
    git clone git@github.com:jmccormac/pySceneNetRGBD.git
    ```

2. Download the SceneNet validation set (15GB) and the validation set protobuf file to the `data` directory of the `pySceneNetRGBD` folder, then run make in the root `pySceneNetRGBD` folder to generate the protobuf description.

    ```bash
    cd pySceneNetRGBD/data
    wget http://www.doc.ic.ac.uk/~ahanda/scenenet_rgbd_val.pb scenenet_rgbd_val.pb
    wget http://www.doc.ic.ac.uk/~ahanda/SceneNetRGBD-val.tar.gz SceneNetRGBD-val.tar.gz
    tar -xvzf SceneNetRGBD-val.tar.gz
    cd .. && make
    ```

3. Run the Python script from this repo writing the SceneNet RGBD data for a trajectory to a rosbag as a sequence of RGB and depth images, coloured pointclouds of the scene, ground truth instance segmentation images, and coloured pointcloudes of ground truth instance segments.

    ```bash
    cd ../scenenet_ros_tools
    python scenenet_to_rosbag.py -scenenet_path SCENENET_PATH -trajectory TRAJECTORY -to_frame TO_FRAME -output_bag OUTPUT_BAG
    ```

    For example:
    ```bash
    python scenenet_to_rosbag.py -scenenet_path ../pySceneNetRGBD -trajectory 1 -output_bag scenenet_traj_1.bag
    ```
    The output bag contains the following topics:
    ```bash
    # RGB and depth images
    /camera/depth/camera_info       : sensor_msgs/CameraInfo
    /camera/depth/image_raw         : sensor_msgs/Image        
    /camera/rgb/camera_info         : sensor_msgs/CameraInfo
    /camera/rgb/image_raw           : sensor_msgs/Image
    # Ground truth instance segmentation image
    /camera/instances/image_raw     : sensor_msgs/Image
    # Cloured pointcloud of ground truth instance segment         
    /scenenet_node/object_segment   : sensor_msgs/PointCloud2
    #  Coloured pointcloud of the scene
    /scenenet_node/scene            : sensor_msgs/PointCloud2
    # Transform from /scenenet_camera_frame to /world
    /tf                             : tf/tfMessage
    ```

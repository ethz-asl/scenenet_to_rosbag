#!/usr/bin/env python
import os
import sys
import argparse
import numpy as np
import rosbag
import rospy
import cv2

from cv_bridge import CvBridge
from geometry_msgs.msg import Point32, TransformStamped
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from tf.msg import tfMessage
import sensor_msgs.point_cloud2 as pc2
import tf


# These functions produce a file path (on Linux systems) to the image given
# a view and render path from a trajectory. As long the data_root_path to the
# root of the dataset is given.  I.e. to either val or train
def photo_path_from_view(render_path, view):
    photo_path = os.path.join(render_path, 'photo')
    image_path = os.path.join(photo_path, '{0}.jpg'.format(view.frame_num))
    return os.path.join(data_root_path, image_path)


def instance_path_from_view(render_path, view):
    photo_path = os.path.join(render_path, 'instance')
    image_path = os.path.join(photo_path, '{0}.png'.format(view.frame_num))
    return os.path.join(data_root_path, image_path)


def depth_path_from_view(render_path, view):
    photo_path = os.path.join(render_path, 'depth')
    image_path = os.path.join(photo_path, '{0}.png'.format(view.frame_num))
    return os.path.join(data_root_path, image_path)


def camera_intrinsic_transform(vfov=45,
                               hfov=60,
                               pixel_width=320,
                               pixel_height=240):
    camera_intrinsics = np.zeros((3, 4))
    camera_intrinsics[2, 2] = 1
    camera_intrinsics[0, 0] = (
        pixel_width / 2.0) / np.tan(np.radians(hfov / 2.0))
    camera_intrinsics[0, 2] = pixel_width / 2.0
    camera_intrinsics[1, 1] = (
        pixel_height / 2.0) / np.tan(np.radians(vfov / 2.0))
    camera_intrinsics[1, 2] = pixel_height / 2.0
    return camera_intrinsics


def get_camera_info():
    camera_intrinsic_matrix = camera_intrinsic_transform()

    camera_info = CameraInfo()
    camera_info.height = 240
    camera_info.width = 320

    camera_info.distortion_model = "plumb_bob"
    camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    camera_info.R = np.ndarray.flatten(np.identity(3))
    camera_info.K = np.ndarray.flatten(camera_intrinsic_matrix[:, :3])
    camera_info.P = np.ndarray.flatten(camera_intrinsic_matrix)

    return camera_info


def normalize(v):
    return v / np.linalg.norm(v)


def world_to_camera_with_pose(view_pose):
    lookat_pose = position_to_np_array(view_pose.lookat)
    camera_pose = position_to_np_array(view_pose.camera)
    up = np.array([0, 1, 0])
    R = np.diag(np.ones(4))
    R[2, :3] = normalize(lookat_pose - camera_pose)
    R[0, :3] = normalize(np.cross(R[2, :3], up))
    R[1, :3] = -normalize(np.cross(R[0, :3], R[2, :3]))
    T = np.diag(np.ones(4))
    T[:3, 3] = -camera_pose
    return R.dot(T)


def camera_to_world_with_pose(view_pose):
    return np.linalg.inv(world_to_camera_with_pose(view_pose))


def position_to_np_array(position):
    return np.array([position.x, position.y, position.z])


def interpolate_poses(start_pose, end_pose, alpha):
    assert alpha >= 0.0
    assert alpha <= 1.0
    camera_pose = alpha * position_to_np_array(end_pose.camera)
    camera_pose += (1.0 - alpha) * position_to_np_array(start_pose.camera)
    lookat_pose = alpha * position_to_np_array(end_pose.lookat)
    lookat_pose += (1.0 - alpha) * position_to_np_array(start_pose.lookat)
    timestamp = interpolate_timestamps(start_pose.timestamp,
                                       end_pose.timestamp, alpha)
    pose = sn.Pose()
    pose.camera.x = camera_pose[0]
    pose.camera.y = camera_pose[1]
    pose.camera.z = camera_pose[2]
    pose.lookat.x = lookat_pose[0]
    pose.lookat.y = lookat_pose[1]
    pose.lookat.z = lookat_pose[2]
    pose.timestamp = timestamp
    return pose


def interpolate_timestamps(start_timestamp, end_timestamp, alpha):
    return alpha * end_timestamp + (1.0 - alpha) * start_timestamp


def writeTransform(view, timestamp, frame_id, output_bag):
    ground_truth_pose = interpolate_poses(view.shutter_open,
                                          view.shutter_close, 0.5)
    scale, shear, angles, transl, persp = tf.transformations.decompose_matrix(
        camera_to_world_with_pose(ground_truth_pose))
    rotation = tf.transformations.quaternion_from_euler(*angles)

    trans = TransformStamped()
    trans.header.stamp = timestamp
    trans.header.frame_id = 'world'
    trans.child_frame_id = frame_id
    trans.transform.translation.x = transl[0]
    trans.transform.translation.y = transl[1]
    trans.transform.translation.z = transl[2]
    trans.transform.rotation.x = rotation[0]
    trans.transform.rotation.y = rotation[1]
    trans.transform.rotation.z = rotation[2]
    trans.transform.rotation.w = rotation[3]

    msg = tfMessage()
    msg.transforms.append(trans)

    output_bag.write('/tf', msg, timestamp)


def pack_bgr(blue, green, red):
    # Pack the 3 BGR channels into a single UINT32 field as RGB.
    return np.bitwise_or(
        np.bitwise_or(
            np.left_shift(red.astype(np.uint32), 16),
            np.left_shift(green.astype(np.uint32), 8)), blue.astype(np.uint32))


def euclidean_ray_length_to_z_coordinate(depth_image, camera_model):
    center_x = camera_model.cx()
    center_y = camera_model.cy()

    constant_x = 1 / camera_model.fx()
    constant_y = 1 / camera_model.fy()

    vs = np.array(
        [(v - center_x) * constant_x for v in range(0, depth_image.shape[1])])
    us = np.array(
        [(u - center_y) * constant_y for u in range(0, depth_image.shape[0])])

    return (np.sqrt(
        np.square(depth_image / 1000.0) /
        (1 + np.square(vs[np.newaxis, :]) + np.square(us[:, np.newaxis]))) *
            1000.0).astype(np.uint16)


def convert_bgrd_to_pcl(bgr_image, depth_image, camera_model):
    center_x = camera_model.cx()
    center_y = camera_model.cy()

    constant_x = 1 / camera_model.fx()
    constant_y = 1 / camera_model.fy()

    pointcloud_xzyrgb_fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
    ]

    vs = np.array(
        [(v - center_x) * constant_x for v in range(0, depth_image.shape[1])])
    us = np.array(
        [(u - center_y) * constant_y for u in range(0, depth_image.shape[0])])

    # Convert depth from cm to m.
    depth_image = depth_image / 1000.0

    x = np.multiply(depth_image, vs)
    y = depth_image * us[:, np.newaxis]

    stacked = np.ma.dstack((x, y, depth_image, bgr_image))
    compressed = stacked.compressed()
    pointcloud = compressed.reshape((int(compressed.shape[0] / 6), 6))

    pointcloud = np.hstack((pointcloud[:, 0:3],
                            pack_bgr(*pointcloud.T[3:6])[:, None]))
    pointcloud = [[point[0], point[1], point[2], point[3]]
                  for point in pointcloud]

    pointcloud = pc2.create_cloud(Header(), pointcloud_xzyrgb_fields,
                                  pointcloud)
    return pointcloud


def mono_to_bgr(mono_image):
    # Representable colors in 24 bits.
    n_total_colors = 2**24
    # Unique colors wanted.
    n_unique_colors = 100

    # Step between two magnitude values.
    delta = n_total_colors / n_unique_colors

    # Prepare 3D shape of BGR image.
    broadcasted_mono_image = np.dstack((mono_image, mono_image, mono_image))

    # Magnitude to packed BGR colors.
    packed_bgr_image = np.multiply(
        np.mod(broadcasted_mono_image, n_unique_colors), delta)

    # Shift by 16 and 8 bits through division.
    blue_bit_shift = 1.0 / 2**16
    green_bit_shift = 1.0 / 2**8

    bit_shifters = np.dstack((np.full(mono_image.shape, blue_bit_shift),
                              np.full(mono_image.shape, green_bit_shift),
                              np.full(mono_image.shape, 1)))

    # Packed BGR values shifted by correct amount.
    shifted_bgr_image = np.multiply(packed_bgr_image, bit_shifters).astype(
        np.uint8)

    # Apply mask to get final BGR values.
    bgr_image = np.bitwise_and(shifted_bgr_image, np.array([0x0000ff])).astype(
        np.uint8)
    return bgr_image


def convert(scenenet_path, trajectory, to_frame, output_bag):
    rospy.init_node('scenenet_node', anonymous=True)
    frame_id = "/scenenet_camera_frame"

    # Write RGB and depth images.
    write_rgbd = True
    # Write instance image.
    write_instances = True
    # Write colorized instance image.
    write_instances_color = False
    # Write colored pointclouds of the instance segments.
    write_object_segments = False
    # Write colored pointclouds of the whole scene.
    write_scene_pcl = False

    # Set camera information and model.
    camera_info = get_camera_info()
    camera_model = PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info)

    # Initialize some vars.
    header = Header(frame_id=frame_id)
    cvbridge = CvBridge()
    trajectories = sn.Trajectories()

    # Read all trajectories from the protobuf file.
    try:
        with open(protobuf_path, 'rb') as f:
            trajectories.ParseFromString(f.read())
    except IOError:
        print('SceneNet protobuf data not found at location: {0}'.format(
            protobuf_path))
        sys.exit(
            'Please ensure you have downloaded the protobuf file to the data directory.'
        )

    # Unfortunately, the trajectory indices do not match to their render_path,
    # so trajectory with index 3 could map to the folder 0/123 in
    # your data/val folder.
    # One can choose the folder with the desired trajectory and identify the
    # corresponding trajectory index by checking the "render_path" field.
    traj = trajectories.trajectories[trajectory]
    print('Writing trajectory from location: ' + format(traj.render_path))
    '''
    The views attribute of trajectories contains all of the information
    about the rendered frames of a scene.  This includes camera poses,
    frame numbers and timestamps.
    '''
    view_idx = 0
    while (not rospy.is_shutdown() and view_idx < (to_frame)
           and view_idx < len(traj.views)):
        view = traj.views[view_idx]
        timestamp = rospy.Time(
            interpolate_timestamps(view.shutter_open.timestamp,
                                   view.shutter_close.timestamp, 0.5))
        writeTransform(view, timestamp, frame_id, output_bag)
        header.stamp = timestamp

        # Read RGB, Depth and Instance images for the current view.
        photo_path = photo_path_from_view(traj.render_path, view)
        depth_path = depth_path_from_view(traj.render_path, view)
        instance_path = instance_path_from_view(traj.render_path, view)
        if not os.path.exists(photo_path):
            print("SceneNet RGB-D data not found at {0}".format(photo_path))
            sys.exit("Please ensure you have downloaded the trajectory data.")

        bgr_image = cv2.imread(photo_path, cv2.IMREAD_COLOR)
        depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        instance_image = cv2.imread(instance_path, cv2.IMREAD_UNCHANGED)

        # Transform depth values from the Euclidean ray length to the z coordinate.
        depth_image = euclidean_ray_length_to_z_coordinate(
            depth_image, camera_model)

        if (write_object_segments):
            # Write all the instances in the current view as pointclouds.
            instances_in_current_frame = np.unique(instance_image)

            for instance in instances_in_current_frame:
                instance_mask = np.ma.masked_not_equal(instance_image,
                                                       instance).mask
                masked_depth_image = np.ma.masked_where(
                    instance_mask, depth_image)

                # Workaround for when 2D mask is only False values and collapses to a single boolean False.
                if (not instance_mask.any()):
                    instance_mask_3D = np.broadcast_arrays(
                        instance_mask[np.newaxis, np.newaxis, np.newaxis],
                        bgr_image)
                else:
                    instance_mask_3D = np.broadcast_arrays(
                        instance_mask[:, :, np.newaxis], bgr_image)

                masked_bgr_image = np.ma.masked_where(instance_mask_3D[0],
                                                      bgr_image)

                object_segment_pcl = convert_bgrd_to_pcl(
                    masked_bgr_image, masked_depth_image, camera_model)
                object_segment_pcl.header = header
                output_bag.write('/scenenet_node/object_segment',
                                 object_segment_pcl, timestamp)

        if (write_scene_pcl):
            # Write the scene for the current view as pointcloud.
            scene_pcl = convert_bgrd_to_pcl(bgr_image, depth_image,
                                            camera_model)
            scene_pcl.header = header
            output_bag.write('/scenenet_node/scene', scene_pcl, timestamp)

        if (write_rgbd):
            # Write the RGBD data.
            bgr_msg = cvbridge.cv2_to_imgmsg(bgr_image, "bgr8")
            bgr_msg.header = header
            output_bag.write('/camera/rgb/image_raw', bgr_msg, timestamp)

            depth_msg = cvbridge.cv2_to_imgmsg(depth_image, "16UC1")
            depth_msg.header = header
            output_bag.write('/camera/depth/image_raw', depth_msg, timestamp)

            camera_info.header = header

            output_bag.write('/camera/rgb/camera_info', camera_info, timestamp)
            output_bag.write('/camera/depth/camera_info', camera_info,
                             timestamp)

        if (write_instances):
            # Write the instance data.
            instance_msg = cvbridge.cv2_to_imgmsg(instance_image, "16UC1")
            instance_msg.header = header

            output_bag.write('/camera/instances/image_raw', instance_msg,
                             timestamp)

        if (write_instances_color):
            # Write the instance data colorized.
            instance_image_bgr = mono_to_bgr(instance_image)
            instance_bgr_msg = cvbridge.cv2_to_imgmsg(instance_image_bgr,
                                                      "bgr8")
            instance_bgr_msg.header = header

            output_bag.write('/camera/instances/image_rgb', instance_bgr_msg,
                             timestamp)

        print("Dataset timestamp: " + '{:4}'.format(timestamp.secs) + "." +
              '{:09}'.format(timestamp.nsecs) + "     Frame: " +
              '{:3}'.format(view_idx + 1) + " / " + str(len(traj.views)))

        view_idx += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        usage='''%(prog)s [-h] --scenenet-path PATH --dataset-type {train,val}
                             --trajectory INDEX [--train-set-split N]
                             [--limit NUM] [--output-bag NAME]''',
        description='Convert data from a SceneNet RGB-D trajectory to a rosbag.'
    )
    parser.add_argument(
        "--scenenet-path",
        required=True,
        help="path to the pySceneNetRGBD folder",
        metavar="PATH")
    parser.add_argument(
        "--dataset-type",
        required=True,
        choices=['train', 'val'],
        help="select the train or val dataset")
    parser.add_argument(
        "--train-set-split",
        choices=[str(i) for i in range(17)],
        help="select the N-th training set split",
        metavar="N")
    parser.add_argument(
        "--trajectory",
        type=int,
        required=True,
        help="select the trajectory with index INDEX",
        metavar="INDEX")
    parser.add_argument(
        "--limit",
        default=np.inf,
        type=int,
        help="only write NUM frames to the bag (Default: infinite)",
        metavar="NUM")
    parser.add_argument(
        "--output-bag",
        default="scenenet.bag",
        help="write to bag with name NAME.bag.",
        metavar="NAME")

    args = parser.parse_args()
    scenenet_path = args.scenenet_path
    dataset_type = args.dataset_type
    trajectory = args.trajectory
    output_bag_path = args.output_bag
    to_frame = args.limit

    if dataset_type == "train":
        if not args.train_set_split:
            parser.error("argument --train-set-split is " \
                         "required when --dataset-type=train.")
        dataset_type += "_" + args.train_set_split

    # Include the pySceneNetRGBD folder to the path and import its modules.
    sys.path.append(scenenet_path)
    import scenenet_pb2 as sn

    # Training and validation datasets are stored in pySceneNetRGB/data/train[0-16] and pySceneNetRGB/data/val, respectively.
    data_root_path = os.path.join(scenenet_path, 'data/' + dataset_type)
    protobuf_path = os.path.join(
        scenenet_path, 'data/scenenet_rgbd_{}.pb'.format(dataset_type))

    if not output_bag_path.endswith(".bag"):
        output_bag_path = output_bag_path + ".bag"
    output_bag = rosbag.Bag(output_bag_path, 'w')
    try:
        convert(scenenet_path, trajectory, to_frame, output_bag)
    except rospy.ROSInterruptException:
        pass
    finally:
        output_bag.close()

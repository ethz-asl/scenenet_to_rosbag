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

NYU_13_CLASSES = [(0, 'Unknown'), (1, 'Bed'), (2, 'Books'), (3, 'Ceiling'),
                  (4, 'Chair'), (5, 'Floor'), (6, 'Furniture'), (7, 'Objects'),
                  (8, 'Picture'), (9, 'Sofa'), (10, 'Table'), (11, 'TV'),
                  (12, 'Wall'), (13, 'Window')]

NYU_WNID_TO_CLASS = {
    '04593077': 4,
    '03262932': 4,
    '02933112': 6,
    '03207941': 7,
    '03063968': 10,
    '04398044': 7,
    '04515003': 7,
    '00017222': 7,
    '02964075': 10,
    '03246933': 10,
    '03904060': 10,
    '03018349': 6,
    '03786621': 4,
    '04225987': 7,
    '04284002': 7,
    '03211117': 11,
    '02920259': 1,
    '03782190': 11,
    '03761084': 7,
    '03710193': 7,
    '03367059': 7,
    '02747177': 7,
    '03063599': 7,
    '04599124': 7,
    '20000036': 10,
    '03085219': 7,
    '04255586': 7,
    '03165096': 1,
    '03938244': 1,
    '14845743': 7,
    '03609235': 7,
    '03238586': 10,
    '03797390': 7,
    '04152829': 11,
    '04553920': 7,
    '04608329': 10,
    '20000016': 4,
    '02883344': 7,
    '04590933': 4,
    '04466871': 7,
    '03168217': 4,
    '03490884': 7,
    '04569063': 7,
    '03071021': 7,
    '03221720': 12,
    '03309808': 7,
    '04380533': 7,
    '02839910': 7,
    '03179701': 10,
    '02823510': 7,
    '03376595': 4,
    '03891251': 4,
    '03438257': 7,
    '02686379': 7,
    '03488438': 7,
    '04118021': 5,
    '03513137': 7,
    '04315948': 7,
    '03092883': 10,
    '15101854': 6,
    '03982430': 10,
    '02920083': 1,
    '02990373': 3,
    '03346455': 12,
    '03452594': 7,
    '03612814': 7,
    '06415419': 7,
    '03025755': 7,
    '02777927': 12,
    '04546855': 12,
    '20000040': 10,
    '20000041': 10,
    '04533802': 7,
    '04459362': 7,
    '04177755': 9,
    '03206908': 7,
    '20000021': 4,
    '03624134': 7,
    '04186051': 7,
    '04152593': 11,
    '03643737': 7,
    '02676566': 7,
    '02789487': 6,
    '03237340': 6,
    '04502670': 7,
    '04208936': 7,
    '20000024': 4,
    '04401088': 7,
    '04372370': 12,
    '20000025': 4,
    '03956922': 7,
    '04379243': 10,
    '04447028': 7,
    '03147509': 7,
    '03640988': 7,
    '03916031': 7,
    '03906997': 7,
    '04190052': 6,
    '02828884': 4,
    '03962852': 1,
    '03665366': 7,
    '02881193': 7,
    '03920867': 4,
    '03773035': 12,
    '03046257': 12,
    '04516116': 7,
    '00266645': 7,
    '03665924': 7,
    '03261776': 7,
    '03991062': 7,
    '03908831': 7,
    '03759954': 7,
    '04164868': 7,
    '04004475': 7,
    '03642806': 7,
    '04589593': 13,
    '04522168': 7,
    '04446276': 7,
    '08647616': 4,
    '02808440': 7,
    '08266235': 10,
    '03467517': 7,
    '04256520': 9,
    '04337974': 7,
    '03990474': 7,
    '03116530': 6,
    '03649674': 4,
    '04349401': 7,
    '01091234': 7,
    '15075141': 7,
    '20000028': 9,
    '02960903': 7,
    '04254009': 7,
    '20000018': 4,
    '20000020': 4,
    '03676759': 11,
    '20000022': 4,
    '20000023': 4,
    '02946921': 7,
    '03957315': 7,
    '20000026': 4,
    '20000027': 4,
    '04381587': 10,
    '04101232': 7,
    '03691459': 7,
    '03273913': 7,
    '02843684': 7,
    '04183516': 7,
    '04587648': 13,
    '02815950': 3,
    '03653583': 6,
    '03525454': 7,
    '03405725': 6,
    '03636248': 7,
    '03211616': 11,
    '04177820': 4,
    '04099969': 4,
    '03928116': 7,
    '04586225': 7,
    '02738535': 4,
    '20000039': 10,
    '20000038': 10,
    '04476259': 7,
    '04009801': 11,
    '03909406': 12,
    '03002711': 7,
    '03085602': 11,
    '03233905': 6,
    '20000037': 10,
    '02801938': 7,
    '03899768': 7,
    '04343346': 7,
    '03603722': 7,
    '03593526': 7,
    '02954340': 7,
    '02694662': 7,
    '04209613': 7,
    '02951358': 7,
    '03115762': 9,
    '04038727': 6,
    '03005285': 7,
    '04559451': 7,
    '03775636': 7,
    '03620967': 10,
    '02773838': 7,
    '20000008': 6,
    '04526964': 7,
    '06508816': 7,
    '20000009': 6,
    '03379051': 7,
    '04062428': 7,
    '04074963': 7,
    '04047401': 7,
    '03881893': 13,
    '03959485': 7,
    '03391301': 7,
    '03151077': 12,
    '04590263': 13,
    '20000006': 1,
    '03148324': 6,
    '20000004': 1,
    '04453156': 7,
    '02840245': 2,
    '04591713': 7,
    '03050864': 7,
    '03727837': 5,
    '06277280': 11,
    '03365592': 5,
    '03876519': 8,
    '03179910': 7,
    '06709442': 7,
    '03482252': 7,
    '04223580': 7,
    '02880940': 7,
    '04554684': 7,
    '20000030': 9,
    '03085013': 7,
    '03169390': 7,
    '04192858': 7,
    '20000029': 9,
    '04331277': 4,
    '03452741': 7,
    '03485997': 7,
    '20000007': 1,
    '02942699': 7,
    '03231368': 10,
    '03337140': 7,
    '03001627': 4,
    '20000011': 6,
    '20000010': 6,
    '20000013': 6,
    '04603729': 10,
    '20000015': 4,
    '04548280': 12,
    '06410904': 2,
    '04398951': 10,
    '03693474': 9,
    '04330267': 7,
    '03015149': 9,
    '04460038': 7,
    '03128519': 7,
    '04306847': 7,
    '03677231': 7,
    '02871439': 6,
    '04550184': 6,
    '14974264': 7,
    '04344873': 9,
    '03636649': 7,
    '20000012': 6,
    '02876657': 7,
    '03325088': 7,
    '04253437': 7,
    '02992529': 7,
    '03222722': 12,
    '04373704': 4,
    '02851099': 13,
    '04061681': 10,
    '04529681': 7,
}


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


def publishTransform(view, timestamp, frame_id, output_bag):
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


def pack_bgr(red, green, blue):
    # Pack the 3 RGB channels into a single INT field.
    return np.bitwise_or(
        np.bitwise_or(
            np.left_shift(blue.astype(np.int64), 16),
            np.left_shift(green.astype(np.int64), 8)), red.astype(np.int64))


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


def convert_rgbd_to_pcl(rgb_image, depth_image, camera_model, class_label=0):
    center_x = camera_model.cx()
    center_y = camera_model.cy()

    constant_x = 1 / camera_model.fx()
    constant_y = 1 / camera_model.fy()

    pointcloud_xzyrgb_fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        PointField(
            name='label', offset=16, datatype=PointField.UINT8, count=1)
    ]

    vs = np.array(
        [(v - center_x) * constant_x for v in range(0, depth_image.shape[1])])
    us = np.array(
        [(u - center_y) * constant_y for u in range(0, depth_image.shape[0])])

    # Convert depth from cm to m.
    depth_image = depth_image / 1000.0

    x = np.multiply(depth_image, vs)
    y = depth_image * us[:, np.newaxis]

    stacked = np.ma.dstack((x, y, depth_image, rgb_image))
    compressed = stacked.compressed()
    pointcloud = compressed.reshape((int(compressed.shape[0] / 6), 6))

    pointcloud = np.hstack((pointcloud[:, 0:3],
                            pack_bgr(*pointcloud.T[3:6])[:, None]))
    pointcloud = [[point[0], point[1], point[2], point[3], class_label]
                  for point in pointcloud]

    pointcloud = pc2.create_cloud(Header(), pointcloud_xzyrgb_fields,
                                  pointcloud)
    return pointcloud


def mono_to_rgb(mono_image):
    # Representable colors in 24 bits.
    n_total_colors = 2**24
    # Unique colors wanted.
    n_unique_colors = 70

    # RGB step between two magnitude values.
    delta = n_total_colors / n_unique_colors

    # Prepare 3D shape of RGB image.
    broadcasted_mono_image = np.dstack((mono_image, mono_image, mono_image))

    # Magnitude to packed RGB colors.
    packed_rgb_image = np.multiply(
        np.mod(broadcasted_mono_image, n_unique_colors), delta)

    # Shift by 16 and 8 bits through division.
    red_bit_shift = 1.0 / 2**16
    green_bit_shift = 1.0 / 2**8

    bit_shifters = np.dstack((np.full(mono_image.shape, red_bit_shift),
                              np.full(mono_image.shape, green_bit_shift),
                              np.full(mono_image.shape, 1)))

    # Packed RGB values shifted by correct amount.
    shifted_rgb_image = np.multiply(packed_rgb_image, bit_shifters).astype(
        np.uint8)

    # Apply mask to get final RGB values.
    rgb_image = np.bitwise_and(shifted_rgb_image, np.array([0x0000ff])).astype(
        np.uint8)
    return rgb_image


def publish(scenenet_path, trajectory, output_bag, to_frame):
    rospy.init_node('scenenet_node', anonymous=True)
    frame_id = "/scenenet_camera_frame"

    publish_object_segments = True
    publish_scene_pcl = False
    publish_rgbd = True
    publish_instances = True

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
        print('Scenenet protobuf data not found at location:{0}'.format(
            data_root_path))
        print(
            'Please ensure you have copied the pb file to the data directory')

    # Unfortunately, the trajectory indices don't match their render_path,
    # so trajectory with index 3 could map to the folder 0/123 in
    # your data/val folder.
    # One can choose the folder with the desired trajectory and identify the
    # corresponding trajectory index by checking the "render_path" field.
    traj = trajectories.trajectories[trajectory]
    print('Publishing trajectory from location: ' + format(traj.render_path))

    # Build map from instance ID to NYU class
    instance_class_map = {}
    for instance in traj.instances:
        instance_type = sn.Instance.InstanceType.Name(instance.instance_type)
        if instance.instance_type != sn.Instance.BACKGROUND:
            instance_class_map[instance.instance_id] = NYU_WNID_TO_CLASS[
                instance.semantic_wordnet_id]
    '''
    The views attribute of trajectories contains all of the information
    about the rendered frames of a scene.  This includes camera poses,
    frame numbers and timestamps.
    '''
    view_idx = 0
    while (not rospy.is_shutdown()) and view_idx < len(
            traj.views) and view_idx < (to_frame + 1):
        view = traj.views[view_idx]
        timestamp = rospy.Time(
            interpolate_timestamps(view.shutter_open.timestamp,
                                   view.shutter_close.timestamp, 0.5))
        publishTransform(view, timestamp, frame_id, output_bag)
        header.stamp = timestamp

        # Read RGB, Depth and Instance images for the current view.
        rgb_image = cv2.imread(
            photo_path_from_view(traj.render_path, view), cv2.IMREAD_COLOR)
        depth_image = cv2.imread(
            depth_path_from_view(traj.render_path, view), cv2.IMREAD_UNCHANGED)
        instance_image = cv2.imread(
            instance_path_from_view(traj.render_path, view),
            cv2.IMREAD_UNCHANGED)

        # Transform depth values from the Euclidean ray length to the z coordinate.
        depth_image = euclidean_ray_length_to_z_coordinate(
            depth_image, camera_model)

        if (publish_object_segments):
            # Publish all the instance in the current view as pointclouds.
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
                        rgb_image)
                else:
                    instance_mask_3D = np.broadcast_arrays(
                        instance_mask[:, :, np.newaxis], rgb_image)

                masked_rgb_image = np.ma.masked_where(instance_mask_3D[0],
                                                      rgb_image)

                class_label = 0
                if (instance != 0):
                    class_label = instance_class_map[instance]

                object_segment_pcl = convert_rgbd_to_pcl(
                    masked_rgb_image, masked_depth_image, camera_model,
                    class_label)
                object_segment_pcl.header = header

                output_bag.write('/scenenet_node/object_segment',
                                 object_segment_pcl, timestamp)

        if (publish_scene_pcl):
            # Publish the scene for the current view as pointcloud.
            scene_pcl = convert_rgbd_to_pcl(rgb_image, depth_image,
                                            camera_model)
            scene_pcl.header = header
            output_bag.write('/scenenet_node/scene', scene_pcl, timestamp)

        if (publish_rgbd):
            # Publish the RGBD data.
            rgb_msg = cvbridge.cv2_to_imgmsg(rgb_image, "bgr8")
            rgb_msg.header = header
            output_bag.write('/camera/rgb/image_raw', rgb_msg, timestamp)
            # publisher_rgb_image.publish(rgb_msg)

            depth_msg = cvbridge.cv2_to_imgmsg(depth_image, "16UC1")
            depth_msg.header = header
            output_bag.write('/camera/depth/image_raw', depth_msg, timestamp)

            camera_info.header = header

            output_bag.write('/camera/rgb/camera_info', camera_info, timestamp)
            output_bag.write('/camera/depth/camera_info', camera_info,
                             timestamp)

        if (publish_instances):
            # Publish the instance data.
            color_instance_image = mono_to_rgb(instance_image)
            color_instance_msg = cvbridge.cv2_to_imgmsg(
                color_instance_image, "bgr8")
            color_instance_msg.header = header
            output_bag.write('/camera/instances/image_raw', color_instance_msg,
                             timestamp)

        print("Dataset timestamp: " + '{:4}'.format(timestamp.secs) + "." +
              '{:09}'.format(timestamp.nsecs) + "     Frame: " +
              '{:3}'.format(view_idx + 1) + " / " + str(len(traj.views)))

        view_idx += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Write SceneNet data to a rosbag.')
    parser.add_argument(
        "-scenenet_path",
        default="../pySceneNetRGBD",
        help="Path to the pySceneNetRGBD folder.")
    parser.add_argument(
        "-trajectory",
        default=1,
        help="SceneNet trajectory number to write to the bag.")
    parser.add_argument(
        "-to_frame",
        default=np.inf,
        help="Number of frames to write to the bag.")
    parser.add_argument(
        "-output_bag",
        default="scenenet.bag",
        help="Path to the output rosbag.")

    args = parser.parse_args()
    if args.scenenet_path:
        scenenet_path = args.scenenet_path
    if args.trajectory:
        trajectory = int(args.trajectory)
    if args.output_bag:
        output_bag_path = args.output_bag
    if args.to_frame:
        to_frame = round(args.to_frame)

    # Include the pySceneNetRGBD folder to the path and import its modules.
    sys.path.append(scenenet_path)
    import scenenet_pb2 as sn

    data_root_path = os.path.join(scenenet_path, 'data/val')
    protobuf_path = os.path.join(scenenet_path, 'data/scenenet_rgbd_val.pb')

    bag = rosbag.Bag(output_bag_path, 'w')
    try:
        publish(scenenet_path, trajectory, bag, to_frame)
    except rospy.ROSInterruptException:
        pass
    finally:
        bag.close()

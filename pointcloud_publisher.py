#!/usr/bin/env python

from __future__ import division
import rospy
import scenenet_pb2 as sn
import os

import numpy as np

import cv2
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import tf

from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from geometry_msgs.msg import Point32
import sensor_msgs.point_cloud2 as pc2


data_root_path = 'data/val'
protobuf_path = 'data/scenenet_rgbd_val.pb'

# These functions produce a file path (on Linux systems) to the image given
# a view and render path from a trajectory.  As long the data_root_path to the
# root of the dataset is given.  I.e. to either val or train
def photo_path_from_view(render_path,view):
    photo_path = os.path.join(render_path,'photo')
    image_path = os.path.join(photo_path,'{0}.jpg'.format(view.frame_num))
    return os.path.join(data_root_path,image_path)

def instance_path_from_view(render_path,view):
    photo_path = os.path.join(render_path,'instance')
    image_path = os.path.join(photo_path,'{0}.png'.format(view.frame_num))
    return os.path.join(data_root_path,image_path)

def depth_path_from_view(render_path,view):
    photo_path = os.path.join(render_path,'depth')
    image_path = os.path.join(photo_path,'{0}.png'.format(view.frame_num))
    return os.path.join(data_root_path,image_path)

def camera_intrinsic_transform(vfov = 45, hfov = 60, pixel_width = 320, pixel_height = 240):
    camera_intrinsics = np.zeros((3,4))
    camera_intrinsics[2,2] = 1
    camera_intrinsics[0,0] = (pixel_width  /2.0) / np.tan(np.radians(hfov / 2.0))
    camera_intrinsics[0,2] = pixel_width / 2.0
    camera_intrinsics[1,1] = (pixel_height / 2.0) / np.tan(np.radians(vfov / 2.0))
    camera_intrinsics[1,2] = pixel_height / 2.0
    return camera_intrinsics

def get_camera_info():
    camera_intrinsic_matrix = camera_intrinsic_transform()

    camera_info = CameraInfo()
    camera_info.height = 240
    camera_info.width = 320

    camera_info.distortion_model = "plumb_bob"
    camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    camera_info.R = np.ndarray.flatten(np.identity(3))
    camera_info.K = np.ndarray.flatten(camera_intrinsic_matrix[:,:3])
    camera_info.P = np.ndarray.flatten(camera_intrinsic_matrix)

    return camera_info

def normalize(v):
    return v / np.linalg.norm(v)

def world_to_camera_with_pose(view_pose):
    lookat_pose = position_to_np_array(view_pose.lookat)
    camera_pose = position_to_np_array(view_pose.camera)
    up = np.array([0,1,0])
    R = np.diag(np.ones(4))
    R[2,:3] = normalize(lookat_pose - camera_pose)
    R[0,:3] = normalize(np.cross(R[2,:3],up))
    R[1,:3] = -normalize(np.cross(R[0,:3],R[2,:3]))
    T = np.diag(np.ones(4))
    T[:3,3] = -camera_pose
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
    timestamp = alpha * end_pose.timestamp + (1.0 - alpha) * start_pose.timestamp
    pose = sn.Pose()
    pose.camera.x = camera_pose[0]
    pose.camera.y = camera_pose[1]
    pose.camera.z = camera_pose[2]
    pose.lookat.x = lookat_pose[0]
    pose.lookat.y = lookat_pose[1]
    pose.lookat.z = lookat_pose[2]
    pose.timestamp = timestamp
    return pose

def publishTranform(view, timestamp, frame_id):
    ground_truth_pose = interpolate_poses(view.shutter_open, view.shutter_close, 0.5)
    scale, shear, angles, transl, persp = tf.transformations.decompose_matrix(camera_to_world_with_pose(ground_truth_pose))
    rotation = tf.transformations.quaternion_from_euler(*angles)

    tf_broadcaster = tf.TransformBroadcaster()
    tf_broadcaster.sendTransform(transl,
                                 rotation,
                                 timestamp,
                                 frame_id,
                                 "world")

# Pack the 3 RGB channels into a single UINT32 field
def pack_rgb(red, green, blue):
    return np.bitwise_or(np.bitwise_or(np.left_shift(red.astype(np.uint8), 16), np.left_shift(green.astype(np.uint8), 8)), blue.astype(np.uint8))

def convert_rgbd_to_pcl(rgb_image, depth_image, camera_model):
    center_x = camera_model.cx()
    center_y = camera_model.cy()

    constant_x = 1 / camera_model.fx();
    constant_y = 1 / camera_model.fy();

    pointcloud_xzyrgb_fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
    ]

    vs = np.array([(v - center_x) * constant_x for v in range (0, rgb_image.shape[1])])
    us = np.array([(u - center_y) * constant_y for u in range (0, rgb_image.shape[0])])

    depth_image = np.sqrt(np.square(depth_image / 1000.0) / (1 + np.square(vs[np.newaxis, :]) + np.square(us[:, np.newaxis])))

    x = np.multiply(depth_image, vs)
    y = depth_image * us[:, np.newaxis]

    stacked = np.ma.dstack((x, y, depth_image, rgb_image))
    compressed = stacked.compressed()
    pointcloud = compressed.reshape((int(compressed.shape[0] / 6), 6))

    pointcloud = [[point[0], point[1], point[2], pack_rgb(point[3], point[4], point[5])] for point in pointcloud]

    pointcloud = pc2.create_cloud(Header(), pointcloud_xzyrgb_fields, pointcloud)
    return pointcloud


def publish():
    rospy.init_node('scenenet_node', anonymous=True)
    frame_id = "/scenenet_camera_frame"
    rate = rospy.Rate(1)

    publish_object_segments = True
    publish_scene_pcl = False
    publish_rgbd = False

    # Choose the wanted trajectory to publish
    published_trajectory = 2

    # RGBD and pointcloud publishers
    if (publish_object_segments):
        publisher_object_segment_pcl = rospy.Publisher('/scenenet_node/object_segment', PointCloud2, queue_size=100)

    if (publish_scene_pcl):
        publisher_scene_pcl = rospy.Publisher('/scenenet_node/scene', PointCloud2, queue_size=100)

    if (publish_rgbd):
        publisher_rgb_image = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=100)
        publisher_depth_image = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=100)

        publisher_rgb_camera_info = rospy.Publisher('/camera/rgb/camera_info', CameraInfo, queue_size=100)
        publisher_depth_camera_info = rospy.Publisher('/camera/depth/camera_info', CameraInfo, queue_size=100)

    # Set camera information and model
    camera_info = get_camera_info()
    camera_model = PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info)

    # Set some globals
    header = Header(frame_id = frame_id)
    cvbridge = CvBridge()
    trajectories = sn.Trajectories()

    # Read all trajectories from the protobuf file
    try:
        with open(protobuf_path,'rb') as f:
            trajectories.ParseFromString(f.read())
    except IOError:
        print('Scenenet protobuf data not found at location:{0}'.format(data_root_path))
        print('Please ensure you have copied the pb file to the data directory')

    traj = trajectories.trajectories[published_trajectory]

    '''
    The views attribute of trajectories contains all of the information
    about the rendered frames of a scene.  This includes camera poses,
    frame numbers and timestamps.
    '''

    view_idx = 0
    while not rospy.is_shutdown() and view_idx != len(traj.views):
        view = traj.views[view_idx]

        timestamp = rospy.Time(view.shutter_close.timestamp)
        publishTranform(view, timestamp, frame_id)

        print("Timestamp: " + str(timestamp.secs) + "." + str(timestamp.nsecs) + "     Frame: " + str(view_idx) + " / " + str(len(traj.views)))
        header.stamp = timestamp

        # Read RGB, Depth and Instance images for the current view
        rgb_image = cv2.imread(photo_path_from_view(traj.render_path,view), cv2.IMREAD_COLOR)
        depth_image = cv2.imread(depth_path_from_view(traj.render_path,view), cv2.IMREAD_UNCHANGED)
        instance_image = cv2.imread(instance_path_from_view(traj.render_path,view), cv2.IMREAD_UNCHANGED)

        if (publish_object_segments):
            # Publish all the instance in the current view as pointclouds
            instances_in_current_frame = np.unique(instance_image)

            for instance in instances_in_current_frame:
                instance_mask = np.ma.masked_not_equal(instance_image, instance).mask
                masked_depth_image = np.ma.masked_where(instance_mask, depth_image)

                # Workaround for when 2D mask is only False values and collapses to a single boolean False
                if (not instance_mask.any()):
                    instance_mask_3D = np.broadcast_arrays(instance_mask[np.newaxis, np.newaxis, np.newaxis], rgb_image)
                else:
                    instance_mask_3D = np.broadcast_arrays(instance_mask[:, :, np.newaxis], rgb_image)

                masked_rgb_image = np.ma.masked_where(instance_mask_3D[0], rgb_image)

                object_segment_pcl = convert_rgbd_to_pcl(masked_rgb_image, masked_depth_image, camera_model)
                object_segment_pcl.header = header
                publisher_object_segment_pcl.publish(object_segment_pcl)

        if (publish_scene_pcl):
            # Publish the scene for the current view as pointcloud
            scene_pcl = convert_rgbd_to_pcl(rgb_image, depth_image, camera_model)
            scene_pcl.header = header
            publisher_scene_pcl.publish(scene_pcl)

        if (publish_rgbd):
            # Publish the RGBD data
            rgb_msg = cvbridge.cv2_to_imgmsg(rgb_image, "bgr8")
            rgb_msg.header = header
            publisher_rgb_image.publish(rgb_msg)

            depth_msg = cvbridge.cv2_to_imgmsg(depth_image, "16UC1")
            depth_msg.header = header
            publisher_depth_image.publish(depth_msg)

            camera_info.header = header

            publisher_rgb_camera_info.publish(camera_info)
            publisher_depth_camera_info.publish(camera_info)

        view_idx += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        publish()

    except rospy.ROSInterruptException:
        pass

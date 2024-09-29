'''
    @file triangle_lab.py
    @authors Shubham Joshi (shubham.joshi@study.thws.de),
    @brief Program to detect ports
    @version 0.1

    @copyright Copyright (c) 2024
'''

from ultralytics import YOLO
import cv2
import os
import time
import numpy as np
import math

import rospy
import cv_bridge
from cv2 import WINDOW_NORMAL
from cv2 import WND_PROP_FULLSCREEN
from cv2 import WINDOW_FULLSCREEN
import ros_numpy
import tf2_ros
from tf import transformations
import sensor_msgs.msg
from MSVC2024_Setup_2024.srv import AddTf2
#from MSVC2024_Setup_2024.srv import GetBoardLocation, GetBoardLocationResponse
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from scipy.spatial.transform import Rotation


def service_callback(req):
    global tfBuffer

    if idsBuf is not None:
        try:
            # Call the PortDetection function
            final_detections = PortDetection(ros_deploy=True, detect=True)

            # Process the final detections
            for detection in final_detections:
                cls, transform_matrix = detection

                # Convert the transformation matrix to a TransformStamped message
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = 'base'
                t.child_frame_id = f'port_{cls}'
                t.transform = ros_numpy.msgify(Transform, transform_matrix)

                # Publish or store the transform
                addTf = rospy.ServiceProxy('/store_tf', AddTf2)
                addTf.call(t, False)

            return final_detections

        except Exception as e:
            print("Error in service_callback:", e)
            return None
    else:
        return None


def portDetector(image, x_min=0, y_min=0):
    model = YOLO('latest_port_det_model.pt')
    results = model(image, save=True)

    ports = []

    for result in results:
        boxes = result.boxes
        for box in boxes:
            cls_num = box.cls.numpy()[0]
            xywh = box.xywh.numpy().tolist()[0]
            x, y, w, h = xywh

            # Determine the class label
            if cls_num == 0:
                cls = "lightning"
            elif cls_num == 1:
                cls = "usb_c"
            else:
                cls = "unknown"

            # Calculate the center point of the bounding box
            center_x = x + w / 2 + x_min
            center_y = y + h / 2 + y_min
            if w > h:
                orientation = 0 # corresponds to horizontal
            else:
                orientation = 90 # corresponds to vertical
            simple_box = [cls, center_x, center_y, orientation]
            ports.append(simple_box)

    return ports


## ROS setup stuff
idsBuf = None
zedBuf = None


def tcpPoseCallback(data):
    global pose
    pose = transformStamped_to_numpy(data)


def transformStamped_to_numpy(msg):
    return np.dot(
        transformations.translation_matrix(np.array([msg.transform.translation.x, msg.transform.translation.y,  msg.transform.translation.z])),
        transformations.quaternion_matrix(np.array([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]))
    )


def idsImgCallBack(data):
    global idsBuf
    idsBuf = data


def zedImgCallBack(data):
    global zedBuf
    zedBuf = data


def imgToWorld(image_coords, z_distance, camera_intrinsics):

    fx, fy = camera_intrinsics[0, 0], camera_intrinsics[1, 1]
    cx, cy = camera_intrinsics[0, 2], camera_intrinsics[1, 2]

    world_coords = []
    for (x, y) in image_coords:
        X = (x - cx) * z_distance / fx
        Y = (y - cy) * z_distance / fy
        Z = z_distance
        world_coords.append([X, Y, Z])

    return np.array(world_coords)


def worldToBase(world_coords, camera_pose):

    world_coords_homogeneous = np.hstack((world_coords, np.ones((world_coords.shape[0], 1))))
    robot_base_coords_homogeneous = camera_pose @ world_coords_homogeneous.T
    robot_base_coords = robot_base_coords_homogeneous[:3, :].T

    return robot_base_coords


def PortDetection(ros_deploy=True, detect=True):

    if ros_deploy:
        try:
            idsImage = bridge.imgmsg_to_cv2(idsBuf, desired_encoding='passthrough')
            tmp = idsImage.copy()
            idsImage = tmp # to make idsImage writable
            #print("idsImage shape: ", idsImage.shape)
            #idsImage = cv2.resize(tmp, (1362, 923))
            # cv_image = bridge.imgmsg_to_cv2(idsBuf, "bgr8")
            # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            crop_x = 0
            crop_y = 0
            crop_w = 1362
            crop_h = 923
            image = idsImage[crop_y:crop_y+crop_h, crop_x:crop_x+crop_w]
            result = portDetector(image, crop_x, crop_y)
            print(result)
            cv2.imwrite('/home/robothon/portDetectionTest.png', idsImage) # for debugging

            z_distance = 0.9
            camera_intrinsics = np.array([[6.59120557e+03, 0.00000000e+00, 2.72294039e+03],
                                          [0.00000000e+00, 6.59485693e+03, 1.83543881e+03],
                                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

            image_coords = [(d[1], d[2]) for d in result]
            world_coords = imgToWorld(image_coords, z_distance, camera_intrinsics)

            cam = transformStamped_to_numpy(tfBuffer.lookup_transform('base', 'ids_cam', rospy.Time()))
            cam[2, 3] += 0.009

            robot_base_coords = worldToBase(world_coords, cam)

            final_detections = []
            for i, detection in enumerate(result):
                cls, _, _, orientation = detection
                x, y, z = robot_base_coords[i]

                # Create a rotation matrix based on the orientation
                if orientation == 0:
                    rot_matrix = np.eye(3)
                else:
                    rot_matrix = Rotation.from_euler('z', 90, degrees=True).as_matrix()

                # Create the transformation matrix for the port
                transform_matrix = np.eye(4)
                transform_matrix[:3, :3] = rot_matrix
                transform_matrix[:3, 3] = [x, y, z]

                final_detections.append([cls, transform_matrix])

            print("Final detections:", final_detections)

            return final_detections

        except Exception as e:
            print("Error in PortDetection:", e)
            return []
    else:
        return None


if __name__ == "__main__":

    rospy.init_node('sift_board_detection')
    idsBuf = None
    zedBuf = None
    rospy.Subscriber("/ids/rgb", sensor_msgs.msg.Image, idsImgCallBack)
    rospy.Subscriber("/zed2/zed_node/rgb_raw/image_raw_color", sensor_msgs.msg.Image, zedImgCallBack)
    s = rospy.Service("/sift_board_detection", GetBoardLocation, service_callback)
    bridge = cv_bridge.CvBridge()
    #br = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(20)

    detection = False
    if(detection == False):
        rospy.spin() #spin infinitely and wait for callbacks


    #print(TaskboardTransformMatrix) # for debugging


if __name__ == "__main__":
    rospy.init_node('port_detection_service')
    service = rospy.Service('port_detection', PortDetectionService, service_callback)
    rospy.spin()

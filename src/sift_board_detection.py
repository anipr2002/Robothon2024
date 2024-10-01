#!/usr/bin/env python
'''
    @file board_detection.py
    @authors Shubham Joshi (shubham.joshi@study.thws.de)
    @brief Program to detect the taskboard
    @version 0.1

    @copyright Copyright (c) 2024
'''
import cv2
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
from msvc2024_setup.srv import AddTf2
from msvc2024_setup.srv import GetBoardLocation, GetBoardLocationResponse
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform

from scipy.spatial.transform import Rotation


## ROS setup stuff
idsBuf = None
# zedBuf = None

import time

def service_callback(req):
    global tfBuffer

    if(idsBuf is not None):
        M, angle, mat = BoardDetection()

        cam = transformStamped_to_numpy(tfBuffer.lookup_transform('base_adj', 'ids_cam', rospy.Time()))
        cam[2,3] += 0.009
        print("cam: ", cam) # for debugging

        tb  = cam @ mat

        tb_euler = Rotation.from_matrix(tb[:3,:3]).as_euler("zyx")

        tb_euler[1] = 0
        tb_euler[2] = 0
        tb[:3,:3] = Rotation.from_euler("zyx", tb_euler).as_matrix()
        tb[2][3] = 0.092 #Z-Position taskboard
        print("tb: ", tb)

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'base_adj'
        t.child_frame_id = 'task_board'
        t.transform = ros_numpy.msgify(Transform, tb)
        addTf = rospy.ServiceProxy('/store_tf', AddTf2)
        addTf.call(t, False)
        angle = angle -180
        print("board angle " +str(angle))
        config = int((angle+45)/90)%4
        print(">> config: " + str(config)) # for debugging

        return GetBoardLocationResponse(True,config)
    else:
        return GetBoardLocationResponse(False,0)


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

# def zedImgCallBack(data):
#     global zedBuf
#     zedBuf = data


num_features = 1000
def SIFTBoardDetector(train_img, test_img, ref_pts, num_features=num_features*0.01, reproj_threshold=5.0, show_output=False):

    # Initialize SIFT detector
    sift = cv2.SIFT_create(
        nfeatures=1000,
        contrastThreshold=0.04,
        edgeThreshold=10,
        sigma=1.2
    )
    ref_pts = np.float32(ref_pts).reshape(-1, 1, 2)
    # Detect keypoints and compute descriptors
    keypoints_template, descriptors_template = sift.detectAndCompute(train_img, None)
    keypoints_new, descriptors_new = sift.detectAndCompute(test_img, None)

    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
    matches = bf.match(descriptors_template, descriptors_new)
    # Sort matches based on their distance
    matches = sorted(matches, key=lambda x: x.distance)
    src_pts = np.float32([keypoints_template[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([keypoints_new[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
    # Compute Homography
    M, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, reproj_threshold)
    # Transform the points coorindates from training image to test image
    pts_test = cv2.perspectiveTransform(ref_pts, M).astype(int).reshape(-1, 2)

     # Calculate taskboard angle
    pt1, pt2 = pts_test[:2]
    dx = pt2[0] - pt1[0]
    dy = pt2[1] - pt1[1]
    angle_rad = math.atan2(dy, dx)
    angle_deg = 360.00 - math.degrees(angle_rad)

    # Display the result
    if show_output:
        ref_pts = ref_pts.astype(int).reshape(-1, 2)
        print("ref_pts: ", ref_pts)
        train_img_with_pts = cv2.polylines(train_img, [np.int32(ref_pts[:4])], True, (255, 0, 0), 2, cv2.LINE_AA)
        for pt in ref_pts[4:]:
            cv2.circle(train_img_with_pts, tuple(pt), 3, (255, 0, 0), -1)
        #cv2.imshow('Train Image with Points', train_img_with_pts)
        print("pts_test: ", pts_test)
        print("angle_deg: ", angle_deg)
        print("Homography Matrix M: ", M)
        test_img_with_pts = cv2.polylines(test_img, [np.int32(pts_test[:4])], True, (255, 0, 0), 2, cv2.LINE_AA)
        for pt in pts_test[4:]:
            cv2.circle(test_img_with_pts, tuple(pt), 3, (255, 0, 0), -1)
        #cv2.imshow('Detected Task Board', test_img_with_pts)
        cv2.imwrite("/home/robothon/Detected_Task_Board.png", test_img_with_pts)
        print("Result image saved in home directory")
        matched_img = cv2.drawMatches(train_img, keypoints_template, test_img, keypoints_new, matches[:num_features], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        #cv2.imshow('Matches', matched_img)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

    return pts_test.astype(int), M, angle_deg


def imageToRealBase(img_pts, image):

    realPositions = np.array([[0.224, 0.120,0.003],   # center of blue button
                            [0.224, 0.105, 0.003],  # center of red button
                            [0.2432, 0.1415,-0.002],  # far contralateral screw hole
                            [0.2432, 0.011,-0.002],  # far ipsilateral screw hole
                            [0.1275, 0.1415, -0.002],  # middle contralateral screwhole
                            [0.1275, 0.011,-0.002],   # middle ipsilateral screwhole
                            [0.011, 0.1415,-0.002],  # close contralateral screwhole
                            [0.011, 0.011,-0.002]]) # close ipsilateral screwhole
    # red dimension x is first value, green dimension y is 2nd value, blue dimension z is 3rd value
    # Measured from the corner closest to the 3d-printed holder for the multimeter probe
    print("scaled image points for pnp ") # for debugging
    img_pts = np.array(scalePoints(img_pts[4:], (5536, 3692), (1362, 923), show_output=True), dtype=np.float32)
    print("img_pts: ", img_pts) # for debugging
    real_pts = []
    real_pts.append([realPositions[0][0], realPositions[0][1], realPositions[0][2]])
    real_pts.append([realPositions[1][0], realPositions[1][1], realPositions[1][2]])
    for i in range(realPositions.shape[0]-2):
        real_pts.append([realPositions[i+2][0], realPositions[i+2][1], realPositions[i+2][2]])
    real_pts = np.array(real_pts, dtype=np.float32)
    print("real_pts: ", real_pts) # for debugging

    # camera intrinsic parameters
    Kmat = np.array([[6.59120557e+03, 0.00000000e+00, 2.72294039e+03],[ 0.00000000e+00, 6.59485693e+03, 1.83543881e+03], [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    Dmat = np.array([0.0,0.0,0.0,0.0,0.0])
    # camera extrinsic parameters
    matCam =  np.array([[-9.99818480e-01, -1.87744792e-02, -3.24429211e-03, -2.85266318e-04],[-1.87919161e-02,  9.99808669e-01,  5.43045494e-03, -5.86241360e-01],[ 3.14171741e-03,  5.49043567e-03, -9.99979992e-01,  9.09037476e-01],[ 0.00000000e+00,  0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])

    _, rvec, tvec = cv2.solvePnP(real_pts, img_pts, Kmat, Dmat, useExtrinsicGuess= False, flags = cv2.SOLVEPNP_EPNP)
    _, rotBoard, transBoard = cv2.solvePnP(real_pts, img_pts, Kmat, Dmat, rvec= rvec, tvec=tvec, useExtrinsicGuess= True ,flags = cv2.SOLVEPNP_ITERATIVE)
    cv2.drawFrameAxes(image, Kmat, Dmat, rotBoard, transBoard, length = 0.1, thickness=5)

    rotmatBoard, _ = cv2.Rodrigues(rotBoard)
    matTask = np.eye(4)
    matTask[:3,:3] = rotmatBoard
    matTask[:3,3:] = transBoard
    print("matTask: ", matTask) # for debugging
    return matTask


def scalePoints(points, target_size, original_size=(5536, 3692), show_output=False):
    original_width, original_height = original_size
    target_width, target_height = target_size
    scale_w = target_width / original_width
    scale_h = target_height / original_height
    scaled_points = points * [scale_w, scale_h]
    scaled_points = np.ceil(scaled_points).astype(int)
    if show_output:
        print("Scaled points: ", scaled_points)
    return scaled_points


def BoardDetection(ros_deploy=True, detect=True):

    # taskboard bounding box on training image, found with the aid of board_detection.py
    taskboard_bb =  np.array([[[396, 634]],
                            [[400, 337]],
                            [[887, 341]],
                            [[885, 639]]])
    picPoints = np.array([[3370.5, 1622.5],
                      [3368.5, 1732.5],
                      [3524.5, 1450.5],
                      [3518.5, 2474.5],
                      [2618.5, 1444.5],
                      [2610.5, 2467.5],
                      [1713.5, 1441.5],
                      [1699.5, 2462.5]]) # in original image size
    img_pts = scalePoints(picPoints, (1362, 923), show_output=True)
    all_pts = np.vstack((taskboard_bb.reshape(-1, 2), img_pts))
    train_pts = all_pts
    print("all_pts: ", all_pts) # for debugging

    train_img= cv2.imread('/home/robothon/Robothon/src/msvc2024_setup/src/idsImageresized.png', cv2.IMREAD_GRAYSCALE)

    if ros_deploy:
        try:
            idsImage = bridge.imgmsg_to_cv2(idsBuf, desired_encoding='passthrough')
            tmp = idsImage.copy()
            print("idsImage shape: ", idsImage.shape)
            idsImage = cv2.resize(tmp, (1362, 923))
            # cv_image = bridge.imgmsg_to_cv2(idsBuf, "bgr8")
            # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            cv2.imwrite('/home/robothon/idsImageTest.png', idsImage) # for debugging
            points, H, angle = SIFTBoardDetector(train_img, idsImage, train_pts, num_features=100, reproj_threshold=5.0, show_output=True) # make show_output=False in production
            print("Homography Matrix: ", H) # for debugging
            print("Taskboard Rotation Angle: ", angle)
            print("points: ", points) # for debugging
            matBoard =  imageToRealBase(points, idsImage)
            return H, angle, matBoard
        except Exception as e:
            print(e)
            rate.sleep()
            return None, None, matBoard
    else:
        test_img = cv2.resize(cv2.imread('board_pic_light2.png', cv2.IMREAD_GRAYSCALE), (train_img.shape[1], train_img.shape[0]))
        #train_pts = all_pts.reshape(-1, 2).tolist()
        train_pts = all_pts
        #cv2.polylines(train_img, [np.int32(train_pts)], True, (255, 0, 0), 2, cv2.LINE_AA)
        #cv2.circle(train_img, tuple(train_pts[0]), 3, (255, 255, 255), 2)
        #cv2.imwrite('board_train_bb.png', train_img)
        points, H, angle = SIFTBoardDetector(train_img, test_img, train_pts, num_features=50, reproj_threshold=5.0, show_output=True)
        print("Homography Matrix: ", H) # for debugging
        print("Taskboard Rotation Angle: ", angle)
        print("points: ", points) # for debugging
        #return imageToRealBase(points, test_img)
        return None, None


if __name__ == "__main__":

    rospy.init_node('sift_board_detection')
    idsBuf = None
    # zedBuf = None
    rospy.Subscriber("/ids/rgb", sensor_msgs.msg.Image, idsImgCallBack)
    # rospy.Subscriber("/zed2/zed_node/rgb_raw/image_raw_color", sensor_msgs.msg.Image, zedImgCallBack)
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

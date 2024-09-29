import cv2
import numpy as np
import math

def SIFTBoardDetector(train_img, test_img, ref_pts, num_features=30, reproj_threshold=5.0, show_output=False):

    # Initialize SIFT detector
    sift = cv2.SIFT_create(
        nfeatures=500,
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
    
    # Display the result
    if show_output:
        ref_pts = ref_pts.astype(int).reshape(-1, 2)
        print("ref_pts: ", ref_pts)
        train_img_with_pts = cv2.polylines(train_img, [np.int32(ref_pts[:4])], True, (255, 0, 0), 2, cv2.LINE_AA)
        for pt in ref_pts[4:]:
            cv2.circle(train_img_with_pts, tuple(pt), 3, (255, 0, 0), -1)
        cv2.imshow('Train Image with Points', train_img_with_pts)
        print("pts_test: ", pts_test)
        test_img_with_pts = cv2.polylines(test_img, [np.int32(pts_test[:4])], True, (255, 0, 0), 2, cv2.LINE_AA)
        for pt in pts_test[4:]:
            cv2.circle(test_img_with_pts, tuple(pt), 3, (255, 0, 0), -1)
        cv2.imshow('Detected Task Board', test_img_with_pts)
        matched_img = cv2.drawMatches(train_img, keypoints_template, test_img, keypoints_new, matches[:num_features], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv2.imshow('Matches', matched_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Calculate taskboard angle
        pt1, pt2 = pts_test[:2]
        dx = pt2[0] - pt1[0]
        dy = pt2[1] - pt1[1]
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)

    return pts_test.astype(int), M, angle_deg


def imageToRealBase(img_pts, image):
    
    realPositions = np.array([[0.2232, 0.1199,0.003],
                              [0.2232, 0.1058, 0.003],
                              [0.1653, 0.0947, 0.002], 
                              [0.22365, 0.0528, 0.014], 
                              [0.2432, 0.1412,-0.002], 
                              [0.2432, 0.0108,-0.002], 
                              [0.127, 0.1412, -0.002], 
                              [0.127, 0.0108,-0.002], 
                              [0.0108, 0.1412,-0.002], 
                              [0.0108, 0.0108,-0.002]])
    img_pts = np.array(img_pts) # update this
    real_pts = np.array([[0,0],[0,0]]) # update this with realPositions

    Kmat = np.array([[6.59120557e+03, 0.00000000e+00, 2.72294039e+03],[ 0.00000000e+00, 6.59485693e+03, 1.83543881e+03], [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]) 
    Dmat = np.array([0.0,0.0,0.0,0.0,0.0])
    _, rotBoard, transBoard = cv2.solvePnP(real_pts, img_pts, Kmat, Dmat, useExtrinsicGuess= True ,flags = cv2.SOLVEPNP_ITERATIVE)
    cv2.drawFrameAxes(image, Kmat, Dmat, rotBoard, transBoard, length = 0.1, thickness=5)

    rotmatBoard, _ = cv2.Rodrigues(rotBoard)
    matTask = np.eye(4)
    matTask[:3,:3] = rotmatBoard
    matTask[:3,3:] = transBoard

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
    
    if ros_deploy:
        print("test mode, no ROS")
        # rospy.init_node('board_detection')
        # idsBuf = None
        # rospy.Subscriber("/ids/rgb", sensor_msgs.msg.Image, idsImgCallBack)
        # s = rospy.Service("/board_detection", GetBoardLocation, service_callback)
        # bridge = cv_bridge.CvBridge()
        # tfBuffer = tf2_ros.Buffer()
        # listener = tf2_ros.TransformListener(tfBuffer)
        # rate = rospy.Rate(20)
        # if not detect:
            # rospy.spin()
        # while not rospy.is_shutdown():
            # if idsBuf is not None and detect == True:
                # try:
                    # cv_image = bridge.imgmsg_to_cv2(idsBuf, "bgr8")
                    # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                    # bbox = SIFTBoardDetector(train_img, cv_image, train_pts, num_features=50, reproj_threshold=5.0, show_output=False)
                    # bbox_msg = BoundingBox()
                    # bbox_msg.xmin = bbox[0][0]
                    # bbox_msg.ymin = bbox[0][1]
                    # bbox_msg.xmax = bbox[2][0]
                    # bbox_msg.ymax = bbox[2][1]
                    # s.publish(bbox_msg)
                # except Exception as e:
                    # print(e)
            # rate.sleep()
    else:
        train_img= cv2.imread('idsImageresized.png', cv2.IMREAD_GRAYSCALE)
        test_img = cv2.resize(cv2.imread('board_pic_light2.png', cv2.IMREAD_GRAYSCALE), (train_img.shape[1], train_img.shape[0]))
        #train_pts = all_pts.reshape(-1, 2).tolist()
        train_pts = all_pts
        #cv2.polylines(train_img, [np.int32(train_pts)], True, (255, 0, 0), 2, cv2.LINE_AA)
        #cv2.circle(train_img, tuple(train_pts[0]), 3, (255, 255, 255), 2)
        #cv2.imwrite('board_train_bb.png', train_img)
        points, H, angle = SIFTBoardDetector(train_img, test_img, train_pts, num_features=50, reproj_threshold=5.0, show_output=True)
        print("Homography Matrix: ", H) # for debugging
        print("Taskboard Rotation Angle: ", angle)
        #return imageToRealBase(points, test_img)
        return None
       

if __name__ == "__main__":
    #Actual setup here - uncomment this and change ros_deploy to True for deploying on ROS
    # rospy.init_node('board_detection')
    # idsBuf = None
    # rospy.Subscriber("/ids/rgb", sensor_msgs.msg.Image, idsImgCallBack)
    # s = rospy.Service("/board_detection", GetBoardLocation, service_callback)

    TaskboardTransformMatrix = BoardDetection(ros_deploy=False, detect=True)
    #print(TaskboardTransformMatrix) # for debugging
import cv2
import numpy as np
import rospy

import cv_bridge
import ros_numpy

import tf2_ros
from tf import transformations

import sensor_msgs.msg
from robothon2023.srv import AddTf2
from robothon2023.srv import GetBoardLocation, GetBoardLocationResponse
from geometry_msgs.msg import TransformStamped 
from geometry_msgs.msg import Transform

from scipy.spatial.transform import Rotation


# rewriting the code as a single usable function
def SIFT_board_detector(train_img, test_img, ref_bbox, num_features=30, reproj_threshold=5.0, show_output=False):

    # Initialize SIFT detector
    sift = cv2.SIFT_create(
        nfeatures=500,
        contrastThreshold=0.04,
        edgeThreshold=10,
        sigma=1.2
    )
    ref_bbox = np.float32(ref_bbox).reshape(-1, 1, 2)
    # Detect keypoints and compute descriptors for the template and new image
    keypoints_template, descriptors_template = sift.detectAndCompute(train_img, None)
    keypoints_new, descriptors_new = sift.detectAndCompute(test_img, None)
    # Use the BFMatcher to match descriptors
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
    matches = bf.match(descriptors_template, descriptors_new)
    # Sort matches based on their distance (best matches first)
    matches = sorted(matches, key=lambda x: x.distance)
    # Extract the matched keypoints' coordinates
    src_pts = np.float32([keypoints_template[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([keypoints_new[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
    # Compute the homography matrix using RANSAC
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, reproj_threshold)
    # Transform the bounding box coordinates from the training image to the test image
    bbox_test = cv2.perspectiveTransform(ref_bbox, M)
    # Draw the bounding box on the test image
    test_img_with_bbox = cv2.polylines(test_img, [np.int32(bbox_test)], True, (255, 0, 0), 2, cv2.LINE_AA)


    # Display the result
    if show_output:
        cv2.imshow('Detected Task Board', test_img_with_bbox)
        matched_img = cv2.drawMatches(train_img, keypoints_template, test_img, keypoints_new, matches[:num_features], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv2.imshow('Matches', matched_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return bbox_test.astype(int)


# rospy stuff ---------------------------------------------------------
### ROS setup stuff
idsBuf = None
zedBuf = None

def service_callback(req):
    global tfBuffer

    if(idsBuf is not None):
        M, angle = BoardDetetction()

        cam = transformStamped_to_numpy(tfBuffer.lookup_transform('base', 'ids_cam', rospy.Time()))
        cam[2,3] += 0.009 
        
        tb  = cam @ M

        tb_euler = Rotation.from_matrix(tb[:3,:3]).as_euler("zyx")

        tb_euler[1] = 0
        tb_euler[2] = 0
        tb[:3,:3] = Rotation.from_euler("zyx", tb_euler).as_matrix()
        tb[2][3] = 0.094 #Z-Position taskboard

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'base'
        t.child_frame_id = 'task_board'
        t.transform = ros_numpy.msgify(Transform, tb)
        addTf = rospy.ServiceProxy('/store_tf', AddTf2)
        addTf.call(t, False)

        print("board angle " +str(angle))
        config = int((angle+45)/90)%4

        return GetBoardLocationResponse(True,config)
    else:
        return GetBoardLocationResponse(False,0)

#def realsenseImgCallback(data):
#    global realBuf
#    realBuf = data

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
# --------------------------------------------------------------------


def calculateRealPoints(bluecircels, redcircels, redconnector, blackcircels):
    
    #newpoints = np.array([[points[0][0][0], points[0][0][1]],[points[1][0][0], points[1][0][1]], [points[2][0][0], points[2][0][1]], [points[3][0][0], points[3][0][1]], [points[4][0][0], points[4][0][1]], [points[5][0][1], points[5][0][1]], [points[6][0][1], points[6][0][1]], [points[7][0][0], points[7][0][1]], [points[8][0][0], points[8][0][1]], [points[9][0][0], points[9][0][1]]])
    #print(newpoints)
    #realPositions = np.array([[0.30445,0.12035,0.003], [0.30445, 0.10635, 0.003], [0.2464, 0.0947, 0.002], [0.3037, 0.053, 0.014], [0.32415,0.14185,-0.002], [0.32415, 0.01055,-0.002], [0.12685, 0.14185, -0.002], [0.12685, 0.01055,-0.002], [0.01055, 0.14185,-0.002], [0.01055, 0.01055,-0.002]])
    #realPositions = np.array([[0.30445,0.12035,0.000], [0.30445, 0.10635, 0.000], [0.2464, 0.0947, 0.000], [0.3037, 0.053, 0.014], [0.32415,0.14185,0.0], [0.32415, 0.01055,0.0], [0.12685, 0.14185, 0.0], [0.12685, 0.01055,0.0], [0.01055, 0.14185,0.0], [0.01055, 0.01055,0.0]])
    
    ################### These values are for the 2023 taskboard detection. I am commenting them out to get correct taskboard tf for 24. realPositions found below commented section for 2024
    
    ################## 2023 values (red M5)
    
    # realPositions = np.array([[0.2232, 0.1199,0.003],   # center of blue button
    #                           [0.2232, 0.1058, 0.003],  # center of red button
    #                           [0.1653, 0.0947, 0.002],  # center of red jack
    #                           [0.22365, 0.0528, 0.014], # M5 center
    #                           [0.2432, 0.1412,-0.002],  # far diagonal screw hole
    #                           [0.2432, 0.0108,-0.002],  # far ipsilateral screw hole
    #                           [0.127, 0.1412, -0.002],  # middle contralateral screwhole
    #                           [0.127, 0.0108,-0.002],   # middle ipsilateral screwhole
    #                           [0.0108, 0.1412,-0.002],  # close contralateral screwhole
    #                           [0.0108, 0.0108,-0.002]]) # close ipsilateral screwhole
    # red dimension is first value, green dimension is 2nd value, blue dimension is 3rd value
    # Measured from the corner closest to the 3d-printed holder for the multimeter probe

#################### 2024 values:
                                                                                                    #Updated?
    realPositions = np.array([[0.224, 0.120,0.003],   # center of blue button                           xx
                              [0.224, 0.105, 0.003],  # center of red button                            xx
                              [0.1665, 0.0947, 0.002],  # center of red jack                            xx
                              [0.2195, 0.0525, 0.014], # M5 center                                      xx
                              [0.2432, 0.1415,-0.002],  # far contralateral screw hole                  xx    
                              [0.2432, 0.011,-0.002],  # far ipsilateral screw hole                     xx
                              [0.1275, 0.1415, -0.002],  # middle contralateral screwhole               xx
                              [0.1275, 0.011,-0.002],   # middle ipsilateral screwhole                  xx
                              [0.011, 0.1415,-0.002],  # close contralateral screwhole                  xx
                              [0.011, 0.011,-0.002]]) # close ipsilateral screwhole                     xx
    # red dimension is first value, green dimension is 2nd value, blue dimension is 3rd value           
    # Measured from the corner closest to the 3d-printed holder for the multimeter probe

    picpoints = []
    realpoints = []
    
    if bluecircels is not None:
        picpoints.append([bluecircels[0][0], bluecircels[0][1]])
        realpoints.append([realPositions[0][0], realPositions[0][1], realPositions[0][2]])
    else:
        print("point blue button not detected for pnp")
    if redcircels is not None:
        picpoints.append([redcircels[0][0], redcircels[0][1]])
        realpoints.append([realPositions[1][0], realPositions[1][1], realPositions[1][2]])
    else:
        print("point red button not detected for pnp")
    """
    if redconnector is not None:
        picpoints.append([redconnector[0][0], redconnector[0][1]])
        realpoints.append([realPositions[2][0], realPositions[2][1], realPositions[2][2]])
    else:
        print("point red connector not detected for pnp")
    
    if redM5 is not None:
        picpoints.append([redM5[0][0], redM5[0][1]])
        realpoints.append([realPositions[3][0], realPositions[3][1], realPositions[3][2]])
    else:
        print("point red M5 not detected for pnp")
    """

    for i in range(np.shape(blackcircels)[0]):
        #print("blackcircles " + str(blackcircels[i]))
        if np.isnan(blackcircels[i][0]) == False:
            picpoints.append([blackcircels[i][0], blackcircels[i][1]])
            realpoints.append([realPositions[i+4][0], realPositions[i+4][1], realPositions[i+4][2]])
        else:
            print("black circle " + str(i+4) + " not detected for pnp")

    outPicPositions = np.array(picpoints)
    print("Pic Points: ", outPicPositions) # for debugging
    outRealPositions = np.array(realpoints)

    if np.shape(outPicPositions)[0] == np.shape(outRealPositions)[0]:
        return outPicPositions, outRealPositions
    else:
        print("worng array dimensions")



def caculatePosition2Base(pic, real, image):
    Kmat = np.array([[6.59120557e+03, 0.00000000e+00, 2.72294039e+03],[ 0.00000000e+00, 6.59485693e+03, 1.83543881e+03], [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]) 
    Dmat = np.array([0.0,0.0,0.0,0.0,0.0])

    #rvecCam = np.array([1.99538456e+000, 1.05741672e+250, -1.77548583e-003])
    #cam position ?? good
    #rvecCam = np.array([1.99538456e+000, 0, 0])
    #tvecCam = np.array([5.78500449e+000, -5.80422371e+002,  9.39442875e+002])

    #matCam =  np.array([[-0.99968356, -0.021374,    0.00264996,-0.01135023],[-0.02138098,  0.99973277,  0.00644169,  0.57806456],[-0.0027896,   0.00639422, -0.99988333,  0.92307161],[ 0., 0., 0., 1.        ]])
    matCam =  np.array([[-9.99818480e-01, -1.87744792e-02, -3.24429211e-03, -2.85266318e-04],[-1.87919161e-02,  9.99808669e-01,  5.43045494e-03, -5.86241360e-01],[ 3.14171741e-03,  5.49043567e-03, -9.99979992e-01,  9.09037476e-01],[ 0.00000000e+00,  0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])
    _, rvec, tvec = cv2.solvePnP(real, pic, Kmat, Dmat, useExtrinsicGuess= False, flags = cv2.SOLVEPNP_EPNP)
    #cv2.drawFrameAxes(image, Kmat, Dmat, rvec, tvec, length = 0.1, thickness=10)

    _, rvecTask, tvecTask = cv2.solvePnP(real, pic, Kmat, Dmat, rvec= rvec, tvec=tvec, useExtrinsicGuess= True ,flags = cv2.SOLVEPNP_ITERATIVE)

    cv2.drawFrameAxes(image, Kmat, Dmat, rvecTask, tvecTask, length = 0.1, thickness=5)

    realCorners = np.array([[0.0,0.0,0], [0.0,0.152,0], [0.254,0.152,0], [0.254,0.0,0]])
    imgCorners, _ = cv2.projectPoints(realCorners, rvecTask, tvecTask, Kmat, Dmat)
    imgCorners = imgCorners.astype(int)
    #print("Image Corners: ", imgCorners) # for debugging
    cv2.line(image,imgCorners[0][0],imgCorners[1][0], [0,255,255],5)
    cv2.line(image,imgCorners[1][0],imgCorners[2][0], [0,255,255],5)
    cv2.line(image,imgCorners[2][0],imgCorners[3][0], [0,255,255],5)
    cv2.line(image,imgCorners[3][0],imgCorners[0][0], [0,255,255],5)
    #print("tvec " + str(tvecTask) + " rvec " +str(rvecTask))
   

    # Taskboard
    rotmatTask, _ = cv2.Rodrigues(rvecTask)
    transmatTask = tvecTask
    matTask = np.eye(4)
    matTask[:3,:3] = rotmatTask
    matTask[:3,3:] = transmatTask
    #matTask = np.linalg.inv(matTask)

    # Camera
    #rotmatCam, _ = cv2.Rodrigues(np.reshape(rvecCam,(3,1))) 
    #print(rotmatCam)
    #transmatCam = np.reshape(tvecCam, (3,1))
    #matCam = np.eye(4)
    #matCam[:3, :3] = rotmatCam
    #matCam[:3, 3:] = transmatCam
    #matCam = np.linalg.inv(matCam)

    #print(matTask)
    #print(matCam)

    #M = matCam @ matTask
   
    #print(M)

    return matTask




def BoardDetection():
    idsImage = bridge.imgmsg_to_cv2(idsBuf, desired_encoding='passthrough')
    # print("shape of IDS image: ", idsImage.shape) # for debugging
    cv2.imwrite('/home/robothon/idsImageresized.png', cv2.resize(idsImage, (1362, 923))) # for debugging
    loadImage = idsImage.copy()
    tmp = idsImage.copy()
    idsImage = tmp

    realPositions = np.array([[0.224, 0.120,0.003],   # center of blue button                           xx
                              [0.224, 0.105, 0.003],  # center of red button                            xx
                              [0.1665, 0.0947, 0.002],  # center of red jack                            xx
                              [0.2195, 0.0525, 0.014], # M5 center                                      xx
                              [0.2432, 0.1415,-0.002],  # far contralateral screw hole                  xx    
                              [0.2432, 0.011,-0.002],  # far ipsilateral screw hole                     xx
                              [0.1275, 0.1415, -0.002],  # middle contralateral screwhole               xx
                              [0.1275, 0.011,-0.002],   # middle ipsilateral screwhole                  xx
                              [0.011, 0.1415,-0.002],  # close contralateral screwhole                  xx
                              [0.011, 0.011,-0.002]]) # close ipsilateral screwhole                     xx
    # red dimension is first value, green dimension is 2nd value, blue dimension is 3rd value           
    # Measured from the corner closest to the 3d-printed holder for the multimeter probe

    picpoints = []
    realpoints = []
    
    if bluecircels is not None:
        picpoints.append([bluecircels[0][0], bluecircels[0][1]])
        realpoints.append([realPositions[0][0], realPositions[0][1], realPositions[0][2]])
    else:
        print("point blue button not detected for pnp")
    if redcircels is not None:
        picpoints.append([redcircels[0][0], redcircels[0][1]])
        realpoints.append([realPositions[1][0], realPositions[1][1], realPositions[1][2]])
    else:
        print("point red button not detected for pnp")
    """
    if redconnector is not None:
        picpoints.append([redconnector[0][0], redconnector[0][1]])
        realpoints.append([realPositions[2][0], realPositions[2][1], realPositions[2][2]])
    else:
        print("point red connector not detected for pnp")
    
    if redM5 is not None:
        picpoints.append([redM5[0][0], redM5[0][1]])
        realpoints.append([realPositions[3][0], realPositions[3][1], realPositions[3][2]])
    else:
        print("point red M5 not detected for pnp")
    """

    for i in range(np.shape(blackcircels)[0]):
        #print("blackcircles " + str(blackcircels[i]))
        if np.isnan(blackcircels[i][0]) == False:
            picpoints.append([blackcircels[i][0], blackcircels[i][1]])
            realpoints.append([realPositions[i+4][0], realPositions[i+4][1], realPositions[i+4][2]])
        else:
            print("black circle " + str(i+4) + " not detected for pnp")

    outPicPositions = np.array(picpoints)
    print("Pic Points: ", outPicPositions) # for debugging
    outRealPositions = np.array(realpoints)

    if np.shape(outPicPositions)[0] == np.shape(outRealPositions)[0]:
        return outPicPositions, outRealPositions
    else:
        print("worng array dimensions")





train_img= cv2.imread('idsImageresized.png', cv2.IMREAD_GRAYSCALE)
test_img = cv2.resize(cv2.imread('board_pic_light1.png', cv2.IMREAD_GRAYSCALE), (train_img.shape[1], train_img.shape[0]))
print(train_img.shape)
taskboard_bb =  np.array([[[396, 634]],
                      [[400, 337]],
                      [[887, 341]],
                      [[885, 639]]])
train_pts = taskboard_bb.reshape(-1, 2).tolist()
cv2.polylines(train_img, [np.int32(train_pts)], True, (255, 0, 0), 2, cv2.LINE_AA)
cv2.circle(train_img, tuple(train_pts[0]), 3, (255, 255, 255), 2)
cv2.imwrite('board_train_bb.png', train_img)


bbox_test = SIFT_board_detector(train_img, test_img, train_pts, num_features=50, reproj_threshold=5.0, show_output=False)
print(bbox_test[0])
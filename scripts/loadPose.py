import cv2
from cv2 import aruco
import rospy
import ros_numpy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, Image

# declare a CvBridge object
br = CvBridge()
PtCloud = None
Img     = None

D =  np.array([0.0, 0.0, 0.0, 0.0, 0.0])
K = np.array([[1360.4704964995865, 0.0, 960.5], [0.0, 1360.4704964995865,540.5], [0.0, 0.0, 1.0]])

def findArucoMarkers(img, markerSize = 6, totalMarkers=250, draw=True):
    markerCorners = None
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    arucoParam = aruco.DetectorParameters_create()
 #   arucoParam.aprilTagMinWhiteBlackDiff = 30 
 #   arucoParam.aprilTagMaxLineFitMse = 20                          
 #   arucoParam.maxErroneousBitsInBorderRate = 0.35                 
 #   arucoParam.errorCorrectionRate = 1.0                             
 #   arucoParam.minMarkerPerimeterRate = 0.20                 
 #   arucoParam.maxMarkerPerimeterRate = 4     
 #   arucoParam.polygonalApproxAccuracyRate = 0.05  
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, markerCorners, parameters = arucoParam)

    #board = cv2.aruco.Board.create()
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, markerCorners, parameters = arucoParam)

    img_ = img.copy()
    
    if ids is not None:
        arr = bboxs[0][0]
        cXcenter = 0
        cYcenter = 0
        aruco.drawDetectedMarkers(img_, bboxs)

        for i in range(arr.shape[0]):
            cX = arr[i,0]
            cY = arr[i,1]
            cXcenter += cX
            cYcenter += cY 
            cv2.circle(img_, (int(cX), int(cY)), 8, (0,0,255), -1)
        
        cXcenter = int(cXcenter/4.0)
        cYcenter = int(cYcenter/4.0)

        cv2.circle(img_, (int(cXcenter), int(cYcenter)), 16, (0,255,0), -1)
        
        rvecs = np.zeros(3)
        tvecs = np.zeros(3)

        pose = cv2.aruco.estimatePoseSingleMarkers(np.array(bboxs), 0.05, K, D)
        #pose = cv2.aruco.estimatePoseBoard(np.array(bboxs), np.array(ids), 0.05, K, D, rvecs, tvecs)
        tvec = pose[1][0][0].tolist()
        print(tvec)
        
        # this is the coordinate of the payload with respect to camera
        # now use the transformation from camera to base-link to convert the pose in body-frame and publish it over TF
        

        img_ = cv2.putText(img_, str(tvec), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA)
    return img_

def imgCallback(msg):
    Img = br.imgmsg_to_cv2(msg)
    
    outImg = findArucoMarkers(Img)
    Img = cv2.resize(Img, (640,480),interpolation = cv2.INTER_NEAREST)
    outImg = cv2.resize(outImg, (640,480), interpolation=cv2.INTER_NEAREST)
    cv2.imshow("Original Image", Img)
    cv2.imshow("Marker", outImg)
    cv2.waitKey(1)

def poseCallback(msg):



def genData():
    rospy.Subscriber('/iris_depth_camera/c920/image_raw',Image, imgCallback)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, poseCallback)
    rospy.spin()

rospy.init_node("publish_pose")
genData()
import cv2
from cv2 import aruco
import rospy
import ros_numpy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PoseStamped

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
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, markerCorners, parameters = arucoParam)

    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, markerCorners, parameters = arucoParam)

    img_ = img.copy()

    aruco.drawDetectedMarkers(img_, bboxs)

    tvec  = np.zeros(3)

    pX = 0
    pY = 0

    if ids is not None:
        for i in range(len(bboxs)):
            cXcenter = 0
            cYcenter = 0
    
            arr = np.array(bboxs[i][0])

            for j in range(arr.shape[0]):
                cX = arr[j,0]
                cY = arr[j,1]
                cXcenter += cX
                cYcenter += cY 
            
            cXcenter = int(cXcenter/4.0)
            cYcenter = int(cYcenter/4.0)
            
            pX += cXcenter
            pY += cYcenter

            cv2.circle(img_, (int(cXcenter), int(cYcenter)), 16, (0,255,0), -1)
            
            pose = cv2.aruco.estimatePoseSingleMarkers(np.array(bboxs[i]), 0.05, K, D)
            tvec += pose[1][0][0]
            
            cv2.circle(img_, (int(cX), int(cY)), 8, (0,0,255), -1)
    
        cv2.circle(img_, ( int(pX/len(bboxs)), int(pY/len(bboxs)) ), 16, (255,0,0), -1)
    
        img_ = cv2.putText(img_, str(tvec), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA)

        loadPose_camFrame = tvec/len(bboxs)
        print(loadPose_camFrame)

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
    pass


def genData():
    rospy.Subscriber('/iris_depth_camera/c920/image_raw',Image, imgCallback)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, poseCallback)
    rospy.spin()

rospy.init_node("publish_pose")
genData()
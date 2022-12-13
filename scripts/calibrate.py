import numpy as np
import cv2
import os
import sys
import rospy
from sensor_msgs.msg import Joy,Image
from gazebo_msgs.srv import GetModelState, GetModelStateRequest,SetModelState
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from gazebo_msgs.msg import ModelState
from cv_bridge import CvBridge

image_list = []
rospy.wait_for_service ('/gazebo/get_model_state')
rospy.wait_for_service('/gazebo/set_model_state')
get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

model = GetModelStateRequest()
model.model_name='calibration_plane'
bridge = CvBridge()

def getmtx(img_list):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objp=np.array([[0,0,0],[1,0,0],[2,0,0],[3,0,0],[0.5,0.5,0],[1.5,0.5,0],[2.5,0.5,0],[3.5,0.5,0]])
    for y in range(2,11):
            for x in range(4):
                    objp=np.append(objp,[np.array([objp[4*(y-2)+x][0],objp[4*(y-2)+x][1]+1,0])],axis=0)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    image_num = 100
    ret0=[]
    j=0
    for i,frame in enumerate(img_list):
        img = frame
        # img = cv2.imread(frame)
        # img=image[::-1]
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findCirclesGrid(gray, (4,11),None,flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
        # If found, add object points, image points (after refining them)
        if ret == True and np.sum(np.int32(ret0))<image_num:
            ret0.append(ret)
            print("{} more for proper calibration".format(image_num-np.sum(np.int32(ret0))))
            objpoints.append(objp.astype('float32'))

            corners2 = cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),criteria)
            imgpoints.append(corners2.reshape(-1, 2).astype('float32'))

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img.copy(), (4,11), corners2,ret)
            cv2.imshow('img',img)
            cv2.waitKey(50)
            # cv2.imwrite('cal{}.jpg'.format(j),img)
            j+=1

        elif np.sum(np.int32(ret0))<image_num:
            cv2.imshow('img',img)
            cv2.waitKey(1)

        else:

            break


    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    print(mtx)
    # h,  w = img.shape[:2]
    # newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

    # print(newcameramtx)

def callback(data):
    global set_state,get_state,model
    orientation = get_state(model).pose.orientation
    loc = get_state(model).pose.position
    x,y,z,w = orientation.x,orientation.y,orientation.z,orientation.w
    r,p,y = (euler_from_quaternion([x,y,z,w]))

    yaw = y
    pitch = p
    roll = r

    if(abs(data.axes[0])>0.3):
        yaw = y-(0.03*data.axes[0])
    if(abs(data.axes[3])>0.3):
        pitch = p-(0.03*data.axes[3])
    if(abs(data.axes[4])>0.3):
        roll = r-(0.03*data.axes[4])

    quat = quaternion_from_euler(roll,pitch,yaw)
    state_msg = ModelState()
    state_msg.model_name = 'calibration_plane'
    state_msg.pose.position.x = loc.x
    state_msg.pose.position.y = loc.y
    state_msg.pose.position.z = loc.z
    state_msg.pose.orientation.x = quat[0]
    state_msg.pose.orientation.y = quat[1]
    state_msg.pose.orientation.z = quat[2]
    state_msg.pose.orientation.w = quat[3]
    set_state( state_msg )

def imgcallback(data):
    cv_img = bridge.imgmsg_to_cv2(data)
    image_list.append(cv_img)

def talker():
    rospy.init_node('calibration_node', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.Subscriber("/car_1/camera/right/image_raw",Image,imgcallback)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        getmtx(image_list)
        pass
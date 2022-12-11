import numpy as np
import cv2
import os
import sys


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp=np.array([[0,0,0],[1,0,0],[2,0,0],[3,0,0],[0.5,0.5,0],[1.5,0.5,0],[2.5,0.5,0],[3.5,0.5,0]])
for y in range(2,11):
        for x in range(4):
                objp=np.append(objp,[np.array([objp[4*(y-2)+x][0],objp[4*(y-2)+x][1]+1,0])],axis=0)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

image_loc = sys.argv[1]
images = os.listdir(image_loc)
images_2 = sorted(([image_loc+x for x in images]))
image_num = 30
ret0=[]
j=0
for i,frame in enumerate(images_2):
    img = cv2.imread(frame)
    # img=image[::-1]
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findCirclesGrid(gray, (4,11),None,flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
    # If found, add object points, image points (after refining them)
    if ret == True and np.sum(np.int32(ret0))<image_num and not i%10:
        ret0.append(ret)
        print("{} more for proper calibration".format(image_num-np.sum(np.int32(ret0))))
        objpoints.append(objp.astype('float32'))

        corners2 = cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),criteria)
        imgpoints.append(corners2.reshape(-1, 2).astype('float32'))

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img.copy(), (4,11), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(100)
        # cv2.imwrite('cal{}.jpg'.format(j),img)
        j+=1

    elif np.sum(np.int32(ret0))<image_num:
        cv2.imshow('img',img)
        cv2.waitKey(1)

    else:

        break


ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
h,  w = img.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

print(newcameramtx)
#!/usr/bin/env python





import numpy as np
import matplotlib.pyplot as plt
import cv2
import os
import math
import matplotlib as mpl
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D




num_of_frames=200
root_dir_path = os.path.dirname(os.path.abspath("__file__"))

#Change directory here
image_dir = os.path.join(root_dir_path, r'/home/kushagra/catkin_ws/src/images/src/dataset1')
images=[]
for i in range(1,num_of_frames+1):
    #Change file name accordingly
    im_name = "{0}/data-{1}.jpg".format(image_dir,str(i))
    images.append(cv2.imread(im_name)[:, :, ::-1])

#Calibration Matrix
k = np.array([[554.254691191187, 0.0, 320.5], 
              [0.0, 554.254691191187, 240.5], 
              [0.0, 0.0, 1.0]], dtype=np.float32)





def extract_features(img):
    
    #Using Clahe for better contrast, thus increasing the number of features detected
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    lab_planes = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0,tileGridSize=(15,15))
    lab_planes[0] = clahe.apply(lab_planes[0])
    lab = cv2.merge(lab_planes)
    img = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    
    #Using FAST
    fast= cv2.FastFeatureDetector_create(threshold = 25, nonmaxSuppression = True)
    kp = fast.detect(img)
    kp = np.array([kp[idx].pt for idx in range(len(kp))], dtype = np.float32)
    return kp




def track_features(image_ref, image_cur,ref):
    #Initializing LK parameters
    lk_params = dict(winSize=(21, 21), criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

    kp2, st, err = cv2.calcOpticalFlowPyrLK(image_ref, image_cur, ref, None, **lk_params)
    st=st.reshape(st.shape[0])
    
    kp1=ref[st==1]
    kp2=kp2[st==1]
    
    return kp1, kp2

#To estimate the relative scale between the clouds
def RelativeScale(last_cloud,new_cloud):
    min_idx=min([new_cloud.shape[0],last_cloud.shape[0]])
    p=new_cloud[:min_idx]
    x=np.roll(p,shift = -3)
    p1=last_cloud[:min_idx]
    x1=np.roll(p1,shift = -3)
    d1=np.linalg.norm(p1-x1,axis=-1)
    d=np.linalg.norm(p-x,axis=-1)
    ratio=d1/d
    ratio=np.median(ratio)
    return ratio

#Estimate current cloud
def triangulate_points(rot,trans,kp1,kp2,k):
    t0=np.array([[1,0,0,0], 
                 [0,1,0,0], 
                 [0,0,1,0]])
    t0=k.dot(t0)
    t1=np.hstack((rot,trans))
    t1=k.dot(t1)
    pts1=kp1.reshape(2,-1)
    pts2=kp2.reshape(2,-1)
    cloud=cv2.triangulatePoints(t0,t1,pts1,pts2).reshape(-1,4)[:,:3]
    return cloud





trajectory=[]
threshold=20

#Compute the keypoints for the first set
image_gray_1=cv2.cvtColor(images[0],cv2.COLOR_BGR2GRAY)
kp1 = extract_features(images[0])

image_gray_2=cv2.cvtColor(images[1],cv2.COLOR_BGR2GRAY)
kp2 = extract_features(images[1])

#Use LKT to track the features
kp1,kp2=track_features(image_gray_1, image_gray_2, kp2)

#Calculate Essesntial matrix
E,mask=cv2.findEssentialMat(kp2,kp1,k,cv2.RANSAC, prob=0.999,mask=None)
kp1=kp1[mask.ravel()==1]
kp2=kp2[mask.ravel()==1]

#Obtain rotation and translation for the essential matrix
retval,rmat,trans,mask=cv2.recoverPose(E,kp1,kp2,k)

#Initialize rotation and translation with the first reading
translation = rmat.dot(trans)
rotation = rmat
trajectory.append(translation)

#Compute the cloud to calculate the scale
new_cloud=triangulate_points(rmat,trans,kp1,kp2,k)

i=1
while(i<=len(images)-1):
    
    image_gray_1=image_gray_2
    old_cloud=new_cloud
    kp1=kp2
    image_gray_2 = cv2.cvtColor(images[i], cv2.COLOR_BGR2GRAY)


    #Track features
    kp1,kp2=track_features(image_gray_1, image_gray_2, kp1)
    
    #If the number of features tracked falls below 20 then recompute the keypoints
    if kp1.shape[0]<threshold:
        kp2=extract_features(images[i])
        i=i+1
        continue
        
    #Essential Matrix    
    E,mask=cv2.findEssentialMat(kp2,kp1,k,cv2.RANSAC,prob=0.999, mask=None)
    kp1=kp1[mask.ravel()==1]
    kp2=kp2[mask.ravel()==1]
    
#     print('Frame:'+str(i)+' Features:'+str(kp1.shape[0]))
    
    #Recover translation and rotation
    retval,rmat,trans,mask=cv2.recoverPose(E,kp1,kp2,k)
    
    #Calculate the cloud of the next set
    new_cloud=triangulate_points(rmat,trans,kp1,kp2,k)
    
    #Compare the two clouds to recover the scale factor
    scale= -RelativeScale(old_cloud, new_cloud)
    #Propagate translation and rotation
    translation=translation+scale*rotation.dot(trans)
    rotation=rotation.dot(rmat)
    trajectory.append(translation)
    #If the number of features tracked falls below 20 then recompute the keypoints
    if kp1.shape[0]<threshold:
        kp2=extract_features(images[i])
    i=i+1





trajectory=np.array(trajectory)
x,y,z=[],[],[]
for i in range(0, trajectory.shape[0]):
    x.append(trajectory[i,0,0])
    y.append(trajectory[i,1,0])
    z.append(trajectory[i,2,0])
fig1=plt.subplot(111)
fig1.plot(x,z)
plt.show()








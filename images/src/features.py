#!usr/bin/env python
import numpy as np 
import cv2

def extract_features(img):
    clahe = cv2.createCLAHE(clipLimit=20.0)
    img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img=clahe.apply(img)
    fast= cv2.FastFeatureDetector_create(threshold = 25, nonmaxSuppression = True)
    kp = fast.detect(img)
    kp = np.array([kp[idx].pt for idx in range(len(kp))], dtype = np.float32)
    return kp

def track_features(image_ref, image_cur,ref):
    

    image_ref=cv2.cvtColor(image_ref,cv2.COLOR_BGR2GRAY)
    image_cur=cv2.cvtColor(image_cur,cv2.COLOR_BGR2GRAY)
    
    #Initializing LK parameters
    lk_params = dict(winSize=(15, 15), criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

    kp2, st, err = cv2.calcOpticalFlowPyrLK(image_ref, image_cur, ref, None, **lk_params)
    kp1, st, err = cv2.calcOpticalFlowPyrLK(image_cur, image_ref, kp2, None, **lk_params)
    distance=abs(ref-kp1).max(-1)
    kp2=kp2[distance<30]
    kp1=kp1[distance<30]
    diff=abs(kp1-kp2).max(-1)
    
    return kp1,kp2,np.mean(diff)

def RelativeScale(last_cloud,new_cloud):
    n=new_cloud.shape[0]
    l=last_cloud.shape[0]
    p=new_cloud[:min([n,l])]
    x=np.roll(p,shift = -3)
    p1=last_cloud[:min([n,l])]
    x1=np.roll(p1,shift = -3)
    d1=np.linalg.norm(p1-x1,axis=-1)
    d=np.linalg.norm(p-x,axis=-1)
    ratio=d1/d
    ratio=np.median(ratio)
    return ratio

#Estimate current cloud wrt to current view
def triangulation(R, t, kp0, kp1, K):
    P0 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])
    P0 = K.dot(P0)
    P1 = np.hstack((R, t))
    P1 = K.dot(P1)
    kp0_3d=np.ones((3,kp0.shape[0]))
    kp1_3d=np.ones((3,kp1.shape[0]))
    kp0_3d[0], kp0_3d[1] = kp0[:, 0].copy(), kp0[:, 1].copy()
    kp1_3d[0], kp1_3d[1] = kp1[:, 0].copy(), kp1[:, 1].copy()
    cloud = cv2.triangulatePoints(P0, P1, kp0_3d[:2],kp1_3d[:2])
    cloud=cloud.T
    return cloud[:,:3]
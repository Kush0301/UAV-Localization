#!/usr/bin/env python

import rospy
import cv2
import os
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix, euler_from_matrix
import numpy as np
import os
import sys
from features import *
from cv_bridge import CvBridge

bridge=CvBridge()
motion=0
threshold=1000
pixdiff=5
odom_pub=rospy.Publisher("visual_odom", Odometry, queue_size=10)
k=0
i_pose=Odometry()
odom=Odometry()
image_ref=0
translation=0
rotation=0
new_cloud=0
kp2=0
image_next=0

def publisher(time, translation, rotation):
    
    print("Time:")
    print(time)
    print("Translation")
    print(translation)
    print("Rotation")
    print(rotation)

    global odom_pub,odom

    rotation1=np.hstack((rotation,np.zeros((3,1))))
    rotation1=np.vstack((rotation1,np.array([0,0,0,1]).reshape(1,4)))
    euler=list(euler_from_matrix(rotation1,'sxyz'))
    quat=quaternion_from_euler(euler[0],euler[1],euler[2])
    position=list(translation.reshape(3,))
    np.random.seed(1)
    p_cov=np.random.random((6,6))
    odom.header.stamp.secs=time
    odom.child_frame_id = "base_link"
    odom.header.frame_id = "visual_odom"
    odom.pose.pose.position.x=position[0]
    odom.pose.pose.position.y=position[1]
    odom.pose.pose.position.z=position[2]
    odom.pose.pose.orientation.x=quat[0]
    odom.pose.pose.orientation.y=quat[1]
    odom.pose.pose.orientation.z=quat[2]
    odom.pose.pose.orientation.w=quat[3]
    odom.pose.covariance=list(p_cov.reshape(36,))
    odom_pub.publish(odom)

def initialize(image_ref,ros_image):

    global bridge
    kp1=extract_features(image_ref)
    
    image_next=bridge.imgmsg_to_cv2(ros_image, "bgr8")
    kp1,kp2,diff=track_features(image_ref, image_next, kp1)
    return kp1,kp2,diff


def image_callback(ros_image):
    
    print("Got new image at time:"+str(ros_image.header.stamp.secs))
    
    global odom_pub,motion,pixdiff,bridge,k,i_pose,threshold,rotation,translation,image_ref,image_next,new_cloud,kp2
    
    if motion==0:
        
        kp1,kp2,diff=initialize(image_ref,ros_image)
        
        if diff>=pixdiff:

            motion=1
            print('Motion Detected')
            E,mask=cv2.findEssentialMat(kp2,kp1,k,cv2.RANSAC,prob=0.999,threshold=0.1, mask=None)
            kp1=kp1[mask.ravel()==1]
            kp2=kp2[mask.ravel()==1]    
            print(kp1.shape[0],kp2.shape[0])

            #Recover relative translation and rotation
            retval,rmat,trans,mask=cv2.recoverPose(E,kp1,kp2,k)
    
             #Calculate the cloud of the next set
            new_cloud=triangulation(rmat,trans,kp1,kp2,k)

            print(new_cloud.shape)
            
            #Propagate translation and rotation
            translation=translation+rotation.dot(trans)
            rotation=rotation.dot(rmat)
            
            publisher(ros_image.header.stamp.secs, translation, rotation)


        elif diff<pixdiff:

            print("No motion detected,publishing previous pose")
            i_pose.header.stamp.secs=ros_image.header.stamp.secs
            odom_pub.publish(i_pose)

    elif motion==1:

        
        old_cloud=new_cloud
        kp1=kp2
        image_ref=image_next
        image_next=bridge.imgmsg_to_cv2(ros_image, "bgr8")
        time=ros_image.header.stamp.secs

        kp1,kp2,diff=track_features(image_ref,image_next,kp1)
        print(kp1.shape[0],kp2.shape[0])
        
        if kp1.shape[0]<threshold:
            kp2=extract_features(image_next)


        #Calculate Essential Matrix and propagate inliers and outliers
        E,mask=cv2.findEssentialMat(kp2,kp1,k,cv2.RANSAC,prob=0.999,threshold=0.4, mask=None)
        kp1=kp1[mask.ravel()==1]
        kp2=kp2[mask.ravel()==1]        

        #Recover relative translation and rotation
        retval,rmat,trans,mask=cv2.recoverPose(E,kp1,kp2,k)
    
        #Calculate the cloud of the next set
        new_cloud=triangulation(rmat,trans,kp1,kp2,k)


        #Compare the two clouds to recover the scale factor
        scale= -RelativeScale(old_cloud, new_cloud)
        #Propagate translation and rotation
        translation=translation+scale*rotation.dot(trans)
        rotation=rotation.dot(rmat)

        publisher(time,translation,rotation)

def calibration(camera_info):
    
    global k
    
    k=camera_info.K
    k=np.array(k).reshape(3,3)
    print("Got Calibration matrix:")
    print(k)

def initial_info(initial_pose):

    global i_pose,translation,rotation
    i_pose=initial_pose
    
    pos=[initial_pose.pose.pose.position.x,initial_pose.pose.pose.position.y,initial_pose.pose.pose.position.z]
    x=initial_pose.pose.pose.orientation.x
    y=initial_pose.pose.pose.orientation.y
    z=initial_pose.pose.pose.orientation.z
    w=initial_pose.pose.pose.orientation.w
    euler=list(euler_from_quaternion([x,y,z,w]))
    print("Got initial position")
    odom_pub.publish(initial_pose)
    print("Initial Pose published")

    rotation=euler_matrix(euler[0],euler[1],euler[2],'sxyz')[:3,:3]         #Initial orientation in matrix form
    translation=np.array(pos).reshape(3,1)                                  #Initial pose in array form   
    print("Intial position")
    print(translation)
    print("Intial orientation:")
    print(rotation)

def main(args):

    global odom_pub,image_ref,bridge

    rospy.init_node('visual_odometry', anonymous=True)
    camera_i=rospy.wait_for_message("/webcam/camera_info",CameraInfo)
    calibration(camera_i)
    
    #Initial Position of UAV
    initial_pos=rospy.wait_for_message("/mavros/global_position/local",Odometry)
    initial_info(initial_pos)

    image_ref=rospy.wait_for_message("/webcam/image_raw",Image)
    image_ref=bridge.imgmsg_to_cv2(image_ref, "bgr8")
    print("Got First Image")

    try:
        image_next=rospy.Subscriber("/webcam/image_raw",Image,image_callback,queue_size=100)
        rospy.spin()
        rospy.sleep(10)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)
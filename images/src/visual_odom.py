#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix, euler_from_matrix
import numpy as np
import os
import sys
from features import *
from cv_bridge import CvBridge

odom_pub=rospy.Publisher("visual_odom", Odometry, queue_size=10)
pixdiff=2
threshold=1000
bridge=CvBridge()

def initialize():
	global pixdiff

	#Camera info
	camera_info=rospy.wait_for_message("/webcam/camera_info",CameraInfo)
	k=camera_info.k
	k=np.array(k).reshape(3,3)

	#Initial Position of UAV
	initial_pose=rospy.wait_for_message("/mavros/global_position/local",Odometry)
	pos=[initial_pose.pose.pose.position.x,initial_pose.pose.pose.position.y,initial_pose.pose.pose.position.z]
	x=initial_pose.pose.pose.orientation.x
	y=initial_pose.pose.pose.orientation.y
	z=initial_pose.pose.pose.orientation.z
	w=initial_pose.pose.pose.orientation.w
	euler=list(euler_from_quaternion([x,y,z,w]))

	#Publishing initial pose
	global odom_pub
	odom_pub.publish(initial_pose)

	rotation=euler_matrix(euler[0],euler[1],euler[2],'rxyz') 		  #Initial orientation in matrix form
	translation=np.array(pos)			  #Initial pose in array form	
	
	#Get first image
	image_initial=rospy.wait_for_message("/webcam/image_raw",Image)
	image_initial = bridge.imgmsg_to_cv2(image_initial, "bgr8")
	kp1=extract_features(image_initial)
	diff=0
	rospy.sleep(10)

	#Get Next Image
	while(diff<pixdiff):
		image_next=rospy.wait_for_message("/webcam/image_raw",Image)
		t=image_next.header.time.secs
		image_next=bridge.imgmsg_to_cv2(image_next, "bgr8")
		kp2=extract_features(image_next)

		#Check pixel difference to verify motion
		kp1,kp2,diff=track_features(image_initial, image_next, kp1)
		odom_pub.publish(inital_pose)

	E,mask=cv2.findEssentialMat(kp2,kp1,k,cv2.RANSAC,prob=0.999,threshold=0.1 mask=None)
    kp1=kp1[mask.ravel()==1]
    kp2=kp2[mask.ravel()==1]	

    #Recover relative translation and rotation
    retval,rmat,trans,mask=cv2.recoverPose(E,kp1,kp2,k)
    
    #Calculate the cloud of the next set
    new_cloud=triangulate_points(rmat,trans,kp1,kp2,k)
    

    #Propagate translation and rotation
    translation=translation*rotation.dot(trans)
    rotation=rotation.dot(rmat)
    
    #Publish odometry values
    Odometry odom
    odom.header.stamp=t
    odom.pose.pose.position=translation.tolist()
    odom.pose.pose.orientation=quaternion_from_euler(list(euler_from_matrix(rotation,'rxyz')))
    odom.covariance=p_cov.reshape(36,1).tolist()
	odom_pub.publish(odom)

    print("Translation:",translation)
    print("Rotation:",rotation)


	while not rospy.is_shutdown():
		#Define pose of Odometry type
		global odom_pub
		global bridge
		global threshold
		global pixdiff
		
		Odometry odom

		old_cloud=new_cloud
		kp1=kp2
		image_ref=image_next

		image_next=rospy.wait_for_message("/webcam/image_raw",Image)
		odom.header.stamp=image_next.header.time.secs #Time at which image was taken
		
		image_next=bridge.imgmsg_to_cv2(image_next, "bgr8")
		kp2=extract_features(image_next)

		kp1,kp2,diff=track_features(image_ref,image_next,kp1)

		if diff>pixdiff:
			if kp1.shape[0]<threshold:
				kp2=extract_features(image_next)


	#Calculate Essential Matrix and propagate inliers and outliers
	E,mask=cv2.findEssentialMat(kp2,kp1,k,cv2.RANSAC,prob=0.999,threshold=0.1 mask=None)
    kp1=kp1[mask.ravel()==1]
    kp2=kp2[mask.ravel()==1]		

    #Recover relative translation and rotation
    retval,rmat,trans,mask=cv2.recoverPose(E,kp1,kp2,k)
    
    #Calculate the cloud of the next set
    new_cloud=triangulate_points(rmat,trans,kp1,kp2,k)


    #Compare the two clouds to recover the scale factor
    scale= -RelativeScale(old_cloud, new_cloud)
    #Propagate translation and rotation
    translation=translation+scale*rotation.dot(trans)
    rotation=rotation.dot(rmat)
    
    odom.pose.pose.position=translation.tolist()
    odom.pose.pose.orientation=quaternion_from_euler(list(euler_from_matrix(rotation,'rxyz')))
    odom.covariance=p_cov.reshape(36,1).tolist()
	odom_pub.publish(odom)

	print("Translation:",translation)
    print("Rotation:",rotation)

    if kp1.shape[0]<threshold:
    	kp2=extract_features(image_next)

	
def main(args):
	rospy.init_node('visual_odometry', anonymous=True)
	initialize()
	rospy.spin()
if __name__ == '__main__':
	main(sys.argv)
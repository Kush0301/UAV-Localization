#!/usr/bin/env python

import rospy
import cv2
import os
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import quaternion_from_matrix, quaternion_matrix,inverse_matrix
import numpy as np
import os
import sys
from features import *
from cv_bridge import CvBridge
import tf
from geometry_msgs.msg import Point, Quaternion
from gazebo_msgs.msg import LinkStates
base_link_odom=tf.TransformBroadcaster()
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
cb_matrix=0
cb_pos=Point()
cb_quat=Quaternion()
def publisher(time, translation,rotation, kp2):

    global cb_matrix,odom_pub,odom, base_link_odom
    trans_mat=np.hstack((rotation,translation.reshape(3,1)))
    trans_mat=np.vstack((trans_mat,np.array([0,0,0,1]).reshape(1,4)))
    trans_mat=np.matmul(inverse_matrix(cb_matrix),trans_mat)
    print("Time:")
    print(time)
    print("Translation")
    print(trans_mat[0:3,3])
    print("Rotation")
    print(trans_mat[0:3,0:3])
    
    quat=quaternion_from_matrix(trans_mat)
    position=list(trans_mat[0:3,3].reshape(3,))
    
    if kp2.shape[0]<=1000:
        p_cov=np.random.random((6,6))+5
    elif kp2.shape[0]>1000 and kp2.shape[0]<=1500:
        p_cov=np.random.random((6,6))+4
    
    elif kp2.shape[0]>1500 and kp2.shape[0]<=2000:
        p_cov=np.random.random((6,6))+3
    elif kp2.shape[0]>2000:
        p_cov=np.random.random((6,6))+2
        
    odom.header.stamp.secs=time
    odom.child_frame_id = "base_link"
    odom.header.frame_id = "map"
    odom.pose.pose.position.x=position[0]
    odom.pose.pose.position.y=position[1]
    odom.pose.pose.position.z=position[2]
    odom.pose.pose.orientation.x=quat[0]
    odom.pose.pose.orientation.y=quat[1]
    odom.pose.pose.orientation.z=quat[2]
    odom.pose.pose.orientation.w=quat[3]
    odom.pose.covariance=list(p_cov.reshape(36,))
    odom_pub.publish(odom)
    # base_link_odom.sendTransform((position[0],position[1],position[2]),(quat[0],quat[1],quat[2],quat[3]),time,"base_link","odom")

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
            translation=translation-rotation.dot(trans)
            rotation=rotation.dot(rmat)

            publisher(ros_image.header.stamp.secs, trans_mat,kp2)


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

        publisher(time,translation,rotation,kp2)

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
    print("Got initial position")
    odom_pub.publish(initial_pose)
    print("Initial Pose published")

    rotation=quaternion_matrix([x,y,z,w])[:3,:3]         #Initial orientation in matrix form
    translation=np.array(pos).reshape(3,1)                                  #Initial pose in array form   
    print("Intial position")
    print(translation)
    print("Intial orientation:")
    print(rotation)

def camera_base_link_transform(links):
    link_list=links.name
    c=link_list.index("iris::camera")
    b=link_list.index("iris::iris_demo::iris::base_link")

    base_pos=Point()
    camera_pos=Point()
    base_ort=Quaternion()
    camera_ort=Quaternion()

    camera_pos=links.pose[c].position
    camera_ort=links.pose[c].orientation
    base_pos=links.pose[b].position
    base_ort=links.pose[b].orientation

    global cb_pos,cb_quat
    


    cb_pos,cb_quat=relative_transformation(camera_pos,camera_ort,base_pos,base_ort)

    global cb_matrix
    cb_matrix=quaternion_matrix(cb_quat)
    cb_matrix[0,3]=cb_pos[0]
    cb_matrix[1,3]=cb_pos[1]
    cb_matrix[2,3]=cb_pos[2]

def relative_transformation(pos1,quat1,pos2,quat2):
    
    quat1=[quat1.x,quat1.y,quat1.z,quat1.w]
    quat2=[quat2.x,quat2.y,quat2.z,quat2.w]
    print(quat1,quat2)
    pos1=np.array([pos1.x,pos1.y,pos1.z]).reshape(1,3)
    tran_mat1=quaternion_matrix(quat1)
    tran_mat1[:3,3]=pos1

    pos2=np.array([pos2.x,pos2.y,pos2.z]).reshape(1,3)
    tran_mat2=quaternion_matrix(quat2)
    tran_mat2[:3,3]=pos2

    mat=np.matmul(tran_mat1,inverse_matrix(tran_mat2))
    quat12=quaternion_from_matrix(mat)
    pos12=[mat[0,3],mat[1,3],mat[2,3]]
    return pos12,quat12
def main(args):

    global odom_pub,image_ref,bridge

    rospy.init_node('visual_odometry', anonymous=True)
    camera_i=rospy.wait_for_message("/webcam/camera_info",CameraInfo)
    calibration(camera_i)
    
    #Initial Position of UAV
    initial_pos=rospy.wait_for_message("/mavros/global_position/local",Odometry)
    initial_info(initial_pos)

    links=rospy.wait_for_message('/gazebo/link_states',LinkStates)
    camera_base_link_transform(links)

    image_ref=rospy.wait_for_message("/webcam/image_raw",Image)
    image_ref=bridge.imgmsg_to_cv2(image_ref, "bgr8")
    print("Got First Image")

   

    try:
        rate = rospy.Rate(10.0)
        image_next=rospy.Subscriber("/webcam/image_raw",Image,image_callback,queue_size=100)
        rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)
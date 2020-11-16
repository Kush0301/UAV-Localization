#!/usr/bin/env python

import rospy
import sys
import numpy as np
from gazebo_msgs.msg import LinkStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix, euler_from_matrix
from geometry_msgs.msg import Point, Quaternion,Pose
import tf
ib_pos=Point()
ib_quat=Quaternion()

cb_pos=Point()
cb_quat=Quaternion()
#Returns pose of 1st wrt 2nd
def relative_transformation(pos1,quat1,pos2,quat2):
	
	pos1=np.array([pos1.x,pos1.y,pos1.z]).reshape(1,3)
	euler1=euler_from_quaternion([quat1.x,quat1.y,quat1.z,quat1.w])
	tran_mat1=euler_matrix(euler1[0],euler1[1],euler1[2],'sxyz')
	tran_mat1[:3,3]=pos1

	pos2=np.array([pos2.x,pos2.y,pos2.z]).reshape(1,3)
	euler2=euler_from_quaternion([quat2.x,quat2.y,quat2.z,quat2.w])
	tran_mat2=euler_matrix(euler2[0],euler2[1],euler2[2],'sxyz')
	tran_mat2[:3,3]=pos2

	mat=np.matmul(tran_mat1,np.linalg.inv(tran_mat2))
	euler12=euler_from_matrix(mat)
	quat12=list(quaternion_from_euler(euler12[0],euler12[1],euler12[2]))
	quat12=Quaternion(quat12[0],quat12[1],quat12[2],quat12[3])
	pos12=Point(mat[0,3],mat[1,3],mat[2,3])

	return pos12,quat12

def info(links):
	link_list=links.name
	i=link_list.index("iris::iris_demo::iris::iris/imu_link")
	c=link_list.index("iris::camera")
	b=link_list.index("iris::iris_demo::iris::base_link")

	base_pos=Point()
	# imu_pos=Point()
	camera_pos=Point()
	base_ort=Quaternion()
	# imu_ort=Quaternion()
	camera_ort=Quaternion()

	# imu_pos=links.pose[i].position
	# imu_ort=links.pose[i].orientation
	camera_pos=links.pose[c].position
	camera_ort=links.pose[c].orientation
	base_pos=links.pose[b].position
	base_ort=links.pose[b].orientation

	global ib_pos,ib_quat,cb_pos,cb_quat
	
	# ib_pos,ib_quat=relative_transformation(imu_pos,imu_ort,base_pos,base_ort)

	cb_pos,cb_quat=relative_transformation(camera_pos,camera_ort,base_pos,base_ort)

	

def main(args):
	rospy.init_node('orientation', anonymous=True)
	links=rospy.wait_for_message('/gazebo/link_states',LinkStates)
	# imu_br = tf.TransformBroadcaster()
	camera_br=tf.TransformBroadcaster()
	info(links)

	global ib_pos,ib_quat,cb_pos,cb_quat
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
	
		# imu_br.sendTransform((ib_pos.x,ib_pos.y, ib_pos.z),(ib_quat.x, ib_quat.y,ib_quat.z, ib_quat.w),rospy.Time.now(),"imu_link","base_link")
		# print("Imu Transform Sent")
		camera_br.sendTransform((cb_pos.x,cb_pos.y, cb_pos.z),(cb_quat.x, cb_quat.y,cb_quat.z, cb_quat.w),rospy.Time.now(),"camera_link","base_link")
		# print("Camera Transform Sent")
		rate.sleep()

if __name__=='__main__':
	main(sys.argv)
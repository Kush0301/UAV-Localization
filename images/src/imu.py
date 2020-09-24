#!/usr/bin/env python
import rospy
import pickle
from sensor_msgs.msg import Imu
d={'time':[],'Quaternion':[],'Angular Velocity':[], 'Acceleration':[], 'Orientation Cov':[],'Angular Vel Cov':[],'linear_acc_cov':[]}
def callback(data):
	global d
	d['Orientation Cov'].append(data.orientation_covariance)
	d['Angular Vel Cov'].append(data.angular_velocity_covariance)
	d['linear_acc_cov'].append(data.linear_acceleration_covariance)

	#Time stamp
	t1=data.header.stamp.secs
	print('Time:',t1)
	d['time'].append(t1)
	#Quaternion Orientation
	x=data.orientation.x
	y=data.orientation.y
	z=data.orientation.z
	w=data.orientation.w
	print('Quaternion:', list((x,y,z,w)))
	d['Quaternion'].append(list((x,y,z,w)))
	#Angular velocity
	x_v=data.angular_velocity.x
	y_v=data.angular_velocity.y
	z_v=data.angular_velocity.z
	print('Angular Velocity:', list((x_v,y_v,z_v)))
	d['Angular Velocity'].append(list((x_v,y_v,z_v)))
	#Linear acceleration
	x_a=data.linear_acceleration.x
	y_a=data.linear_acceleration.y
	z_a=data.linear_acceleration.z
	print('Acceleration:',list((x_a,y_a,z_a)))
	d['Acceleration'].append(list((x_a,y_a,z_a)))
	with open('imu_readings.pkl','wb') as f:
		pickle.dump(d,f,pickle.HIGHEST_PROTOCOL)
def listener():
	rospy.init_node('listener', anonymous=True)
	imu_sub=rospy.Subscriber("/mavros/imu/data", Imu, callback)
	rospy.spin()
		

if __name__ == '__main__':
	listener()

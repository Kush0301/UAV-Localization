#!/usr/bin/env python

import rospy
import sys
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped

def pose(pwcs):
	print("Time:"+str(pwcs.header.stamp.secs))
	print("Position")
	print("\tx:"+str(pwcs.pose.pose.position.x))
	print("\ty:"+str(pwcs.pose.pose.position.y))
	print("\tz:"+str(pwcs.pose.pose.position.z))
	print("Orientation")
	print("\tx:"+str(pwcs.pose.pose.orientation.x))
	print("\ty:"+str(pwcs.pose.pose.orientation.y))
	print("\tz:"+str(pwcs.pose.pose.orientation.z))
	print("\tw:"+str(pwcs.pose.pose.orientation.w))

def main(args):
	rospy.init_node("pose",anonymous=True)
	pos=rospy.Subscriber("/robot_pose_ekf/odom_combined",PoseWithCovarianceStamped,pose)
	rospy.spin()

if __name__=="__main__":
	main(sys.argv)
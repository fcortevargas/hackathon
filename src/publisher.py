#!/usr/bin/env python

import rospy
import random
import time
from geometry_msgs.msg import PoseArray
from tf2.transformations import quaternion_from_euler
# from [PATH_TO_VISION_CODE] import get_qr_pose
from aruco import ArucoDetector

def main():

	# Initialize the ROS system and become a node.
	rospy.init_node("publish_qr_pose", anonymous=True)
	
	# Create a publisher object.
	# Here we need to choose the right topic name.
	pub = rospy.Publisher('rsahackathon/poses', PoseArray, queue_size=10)

	detector = ArucoDetector('rtsp://192.168.56.179:8086')

	# Seed the random number generator.
	random.seed(time.time_ns())

	# Loop at 2Hz until the node is shut down.
	rate = rospy.Rate(2)
	while not rospy.is_shutdown():

		# Create and fill in the message randomly. The z-coordinate of 
		# the Point element defaults to zero. Only consider rotation 
		# about z-axis (yaw).
		positions = detector.updatePositions();

		msg = Pose()
		msg.position.x = random.random()
		msg.position.y = 2 * random.random() - 1
		q = quaternion_from_euler(0, 0, random.random())
		msg.orientation.x = q[0]
		msg.orientation.y = q[1]
		msg.orientation.z = q[2]
		msg.orientation.w = q[3]

		# # Here fill in with the code that gets the pose of the QR code 
		# # with opencv.
		# msg = Pose()
		# x, y, theta = get_qr_pose();
		# msg.position.x = x
		# msg.position.y = y
		# q = quaternion_from_euler(0, 0, theta)
		# msg.orientation.x = q[0]
		# msg.orientation.y = q[1]
		# msg.orientation.z = q[2]
		# msg.orientation.w = q[3]

		# Publish the message.
		pub.publish(msg)

		# Send a message to rosout with the details.
		rospy.loginfo(f"Publishing QR positions:"
			f" position=({msg.position.x}, {msg.position.y}"
			f" orientation=({msg.orientation.x}, {msg.orientation.y},"
			f" {msg.orientation.z}, {msg.orientation.w})")

		# Wait until it's time for another iteration.
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

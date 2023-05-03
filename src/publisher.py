#!/usr/bin/env python3

import rospy
import random
import time
from qr_vision.msg import MarkerArray, Marker
# from [PATH_TO_VISION_CODE] import get_qr_pose
from aruco import ArucoDetector
import sys
import argparse

def main(camera=4, frequency=2):

    # Initialize the ROS system and become a node.
    rospy.init_node("publish_qr_pose", anonymous=True)

    # Create a publisher object.
    # Here we need to choose the right topic name.
    pub = rospy.Publisher('rsahackathon/poses', MarkerArray, queue_size=10)

    detector = ArucoDetector(camera)

    rospy.loginfo("Starting calibration...")
    detector.calibrate()
    rospy.loginfo("Calibrated")

    # Loop at 2Hz until the node is shut down.
    rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():

        # Get the positions of the markers (form: { id: (x,y,theta,time) })
        positions = detector.updatePositions()
        # print("Markers: ", positions)

        # Create the ROS message from the list of markers
        msg = MarkerArray()
        if (positions is None): continue
        for id,position in positions.items():
          marker = Marker()
          marker.name = str(id) # TODO: Add a way to give names to the markers
          marker.id = id
          marker.x = round(position[0],3)
          marker.y = round(position[1],3)
          marker.theta = round(position[2],3)
          marker.age = round(time.time() - position[3],3)
          msg.markers.append(marker)
          print("%s : (x=%s, y=%s, θ=%s) - %s" %(marker.id, marker.x, marker.y, marker.theta, marker.age))

        # Publish the message.
        pub.publish(msg)

        # Send a message to rosout with the details.
        rospy.loginfo("Publishing positions")

        # Wait until it's time for another iteration.
        rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog='Hackathon ROS Publisher')
    parser.add_argument('-c', '--camera')
    parser.add_argument('-f', '--frequency')
    args = parser.parse_args()
    try:
        main(int(args.camera), int(args.frequency))
    except rospy.ROSInterruptException:
        pass

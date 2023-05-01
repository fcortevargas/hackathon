#!/usr/bin/env python3

import rospy
import random
import time
from qr_vision.msg import MarkerArray, Marker
# from [PATH_TO_VISION_CODE] import get_qr_pose
from aruco import ArucoDetector

def main():

    # Initialize the ROS system and become a node.
    rospy.init_node("publish_qr_pose", anonymous=True)

    # Create a publisher object.
    # Here we need to choose the right topic name.
    pub = rospy.Publisher('rsahackathon/poses', MarkerArray, queue_size=10)

    detector = ArucoDetector(0)

    # Loop at 2Hz until the node is shut down.
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():

        # Get the positions of the markers (form: { id: (x,y,theta,time) })
        positions = detector.updatePositions()
        print("Markers: ", positions)

        # Create the ROS message from the list of markers
        msg = MarkerArray()
        if (positions is None): continue
        for id,position in positions.items():
          print(id, position)
          marker = Marker()
          marker.name = str(id) # TODO: Add a way to give names to the markers
          marker.id = id
          marker.x = position[0]
          marker.y = position[1]
          marker.theta = position[2]
          msg.markers.append(marker)

        # Publish the message.
        pub.publish(msg)

        # Send a message to rosout with the details.
        rospy.loginfo("Publishing positions")

        # Wait until it's time for another iteration.
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

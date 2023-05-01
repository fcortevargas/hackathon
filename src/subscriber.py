#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose

def callback(msg: Pose):
   rospy.loginfo(f" position=({round(msg.position.x, 2)}, {round(msg.position.y, 2)}"
      f" orientation=({round(msg.orientation.x, 2)}, {round(msg.orientation.y, 2)},"
      f" {round(msg.orientation.z, 2)}, {round(msg.orientation.w, 2)})")

def main():
   # Initialize the ROS system and become a node.
   rospy.init_node("subscriber_node", anonymous=True)
   
   # Create a subscriber object.
   rospy.Subscriber("qr/pose", Pose, callback)

   # Let ROS take over.
   rospy.spin()

if __name__ == '__main__':
   try:
      main()
   except rospy.ROSInterruptException:
      pass
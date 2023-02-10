#include <ros/ros.h>
#include <geometry_msgs/Pose.h> // For geometry_msgs::Pose.
#include <geometry_msgs/Quaternion.h>  // For geometry_msgs::Quaternion.
#include <stdlib.h> // For rand() and RAND_MAX.
#include <tf2/LinearMath/Quaternion.h> // For quaternion calculations.
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For transforming tf2 to message.
// #include <[PATH_TO_VISION_CODE]/[VISION_CODE].h>

int main(int argc, char **argv)
{
	// Initialize the ROS system and become a node.
	ros::init(argc, argv, "publisher_node");
	ros::NodeHandle nh;

	// Create a publisher object.
	ros::Publisher pub = nh.advertise<geometry_msgs::Pose>(
		"qr/pose", 1000); // Here we need to choose the right topic.

	// Seed the random number generator.
	srand(time(0));

	// Loop at 2Hz until the node is shut down.
	ros::Rate rate(2);
	while(ros::ok())
	{
		// Create and fill in the message randomly. The z-coordinate of 
		// the Point element defaults to zero. Only consider rotation 
		// about z-axis (yaw).
		geometry_msgs::Pose msg;
		msg.position.x = double(rand());
		msg.position.y = double(rand());
		tf2::Quaternion q;
		q.setRPY(0, 0, double(rand()));
		geometry_msgs::Quaternion q_msg = tf2::toMsg(q);
		msg.orientation = q_msg;

		// // Here fill in with the code that gets the pose of the QR code 
		// // with opencv.
		// x, y, theta = [NAMESPACE_VISION_CODE]::get_qr_pose();
		// msg.position.x = x;
		// msg.position.y = y;
		// tf2::Quaternion q;
		// q.setRPY(0, 0, theta);
		// q_msg = tf2::toMsg(q);
		// msg.orientation = q_msg;

		// Publish the message.
		pub.publish(msg);

		// Send a message to rosout with the details.
		ROS_INFO_STREAM("Publishing QR positions:"
			<< " position=" << "(" << msg.position.x << ", " << msg.position.y << ")"
			<< " orientation=" << "(" << msg.orientation.x << ", " 
			<< msg.orientation.y << ", "<< msg.orientation.z << ", " 
			<< msg.orientation.w << ")"); 

		// Wait until it's time for another iteration.
		rate.sleep();
	}
}
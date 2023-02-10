#include <ros/ros.h>
#include <geometry_msgs/Pose.h> // For geometry_msgs::Pose.
#include <iomanip> // for std::setprecision and std::fixed

// A callback function. Executed each time a new Pose
// message arrives.
void callback(const geometry_msgs::Pose& msg)
{
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<<<"position=(" << msg.position.x << ", " << msg.position.y << ")"
		<< " orientation=(" << msg.orientation.x << ", " 
		<< msg.orientation.y << ", "<< msg.orientation.z << ", " 
		<< msg.orientation.w << ")"); 
}

int main(int argc, char const **argv)
{
	// Initialize the ROS system and become a node.
	ros::init(argc, argv, "subscriber_node");
	ros::NodeHanlde nh;

	// Create a subscriber object.
	ros::Subscriber sub = nh.subscribe("qr/pose", 1000,
		&callback); // Here we need to choose the right topic.

	// Let ROS take over.
	ros::spin();
}
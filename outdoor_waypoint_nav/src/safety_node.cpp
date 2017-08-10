#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pubVel;

void cmd_vel_CB(const geometry_msgs::Twist vel_msg)
{
	// Publish msg
	pubVel.publish(vel_msg);
}

int main(int argc, char** argv)
{
    // Initialize node
		ros::init(argc, argv, "safety_node"); //initiate node called safety_node
		ros::NodeHandle n;	
		ROS_INFO("Initiated safety_node");
	
	// Initiate publisher to remap topic send end of node message
		pubVel = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    
	//Initiate subscriber
		ros::Subscriber sub_cmd_vel = n.subscribe("/cmd_vel_intermediate", 1000, cmd_vel_CB);
		
	ros::spin();
	return 0;
}

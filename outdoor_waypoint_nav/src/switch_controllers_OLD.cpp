#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>

ros::Publisher pubVel;
geometry_msgs::Twist vel_msg1;
geometry_msgs::Twist vel_msg2;
int controller_num = 1, current_vel = 1;
bool vel1_empty = true, vel2_emtpy = true;

void cmd_vel1_CB(const geometry_msgs::Twist::ConstPtr& vel_msg)
{
	// Save msg and check that it's not empty
	vel_msg1.linear.x = vel_msg->linear.x;
	vel_msg1.angular.x = vel_msg->angular.x;
	vel_msg1.linear.y = vel_msg->linear.y;
	vel_msg1.angular.y = vel_msg->angular.y;
	vel_msg1.linear.z = vel_msg->linear.z;
	vel_msg1.angular.z = vel_msg->angular.z;

	if(vel_msg->linear.x == 0 && vel_msg->angular.z == 0)
	{
		vel1_empty = true;
	}
	else
	{
		vel1_empty = false;
	}
}

void cmd_vel2_CB(const geometry_msgs::Twist::ConstPtr& vel_msg)
{
	// Save msg and check that it's not empty
	vel_msg2.linear.x = vel_msg->linear.x;
	vel_msg2.angular.x = vel_msg->angular.x;
	vel_msg2.linear.y = vel_msg->linear.y;
	vel_msg2.angular.y = vel_msg->angular.y;
	vel_msg2.linear.z = vel_msg->linear.z;
	vel_msg2.angular.z = vel_msg->angular.z;
	if(vel_msg->linear.x == 0 && vel_msg->angular.z == 0)
	{
		vel2_emtpy = true;
	}
	else
	{
		vel2_emtpy = false;
	}
} 

void controller_num_CB(const std_msgs::Int8::ConstPtr& controller_num_msg)
{
	// Save msg
	controller_num = controller_num_msg->data;
} 

int main(int argc, char** argv)
{
    // Initialize node
		ros::init(argc, argv, "switch_controllers_node"); 
		ros::NodeHandle n;	
		ROS_INFO("Initiated switch_controllers_node");
	
	// Initiate publisher to remap topic send end of node message
		pubVel = n.advertise<geometry_msgs::Twist>("/cmd_vel_intermediate",1000);
    
	// Initiate subscribers
		ros::Subscriber sub_cmd_vel1 = n.subscribe("/cmd_vel1", 1000, cmd_vel1_CB);
		ros::Subscriber sub_cmd_vel2 = n.subscribe("/cmd_vel2", 1000, cmd_vel2_CB);
		ros::Subscriber sub_controller_num = n.subscribe("/outdoor_waypoint_nav/controller_in_use", 1000, controller_num_CB);

	// Publish velocity commands of proper move_base controller
		
		// check which velocity should be published
			if(current_vel == 1 && controller_num == 2 && vel2_emtpy == false)
			{
				// this means the second move_base has started spitting our commands so let's switch
				current_vel = 2;
			} // else, stay with 1

			else if(current_vel == 2 && controller_num == 1 && vel1_empty == false)
			{
				// this means 2 was publishing but then 1 started publishing again so switch back to 1
				current_vel = 1;
			} // else stay with 2
		
		// Publish velocity
			if(current_vel == 1)
			{
				pubVel.publish(vel_msg1);
			}

			else if(current_vel == 2)
			{
				pubVel.publish(vel_msg2);
			}

	ros::spin();
	return 0;
}

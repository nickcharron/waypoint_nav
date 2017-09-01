#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

ros::Publisher pubVel;
geometry_msgs::Twist vel_msg1, vel_msg2;

int controller_num = 1, current_vel = 1;
bool vel1_empty = true, vel2_emtpy = true, controller_1_done = false, controller_2_done = false;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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

void controller_1_CB(const std_msgs::Bool::ConstPtr& controller_1_done_msg)
{
	if(controller_1_done_msg->data == true)
	{
		controller_1_done = true;
		controller_num = 2; // if we get a true here, we know to switch the controller once it starts publishing velocities
	}
	else
	{
		controller_1_done = false; // do not switch the controller number
	}
} 

void controller_2_CB(const std_msgs::Bool::ConstPtr& controller_2_done_msg)
{
	if(controller_2_done_msg->data == true)
	{
		controller_2_done = true;
		controller_num = 1;
	}
	else
	{
		controller_2_done = false;
	}
} 

int main(int argc, char** argv)
{
    // Initialize node
		ros::init(argc, argv, "switch_controllers_node"); 
		ros::NodeHandle n;	
		ROS_INFO("Initiated switch_controllers_node");
		ros::Rate rate(60);
		MoveBaseClient ac1("/controller_1/move_base", true);
		MoveBaseClient ac2("/controller_2/move_base", true);

	// Initiate publisher to remap topic send end of node message
		ros::Publisher pubVel = n.advertise<geometry_msgs::Twist>("/cmd_vel_intermediate",1000);
    
	// Initiate subscribers
		ros::Subscriber sub_cmd_vel1 = n.subscribe("/cmd_vel1", 1000, cmd_vel1_CB);
		ros::Subscriber sub_cmd_vel2 = n.subscribe("/cmd_vel2", 1000, cmd_vel2_CB);
		ros::Subscriber sub_controller_1_status = n.subscribe("/controller_1/controller_1_done", 1000, controller_1_CB);
		ros::Subscriber sub_controller_2_status = n.subscribe("/controller_2/controller_2_done", 1000, controller_2_CB);

	// Publish velocity commands of proper move_base controller
		
		while(ros::ok())
		{
			ros::spinOnce();
			// check which velocity should be published
				if(current_vel == 1 && controller_num == 2 && vel2_emtpy == false)
				{
					// this means the second move_base has started spitting our commands so let's switch
					current_vel = 2;
					ROS_INFO("Switching vel cmds from controller 1 to 2");
					ROS_INFO("Canceling controller 1's current goal");
					ac1.cancelAllGoals();

				} // else, stay with 1

				else if(current_vel == 2 && controller_num == 1 && vel1_empty == false)
				{
					// this means 2 was publishing but then 1 started publishing again so switch back to 1
					current_vel = 1;
					ROS_INFO("Switching vel cmds from controller 2 to 1");
					ROS_INFO("Canceling controller 2's current goal");
					ac2.cancelAllGoals();

				} // else stay with 2
		
			// Publish correct velocity
				if(current_vel == 1)
				{
					pubVel.publish(vel_msg1);
				}

				else if(current_vel == 2)
				{
					pubVel.publish(vel_msg2);
				}
				//rate.sleep();
		}
	return 0;
}

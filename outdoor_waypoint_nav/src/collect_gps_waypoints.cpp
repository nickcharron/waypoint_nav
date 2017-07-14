#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <ros/duration.h>
#include <ros/time.h>

bool collect_request = false;
bool continue_collection = true;
double lati_point=0, longi_point=0, lati_last=0, longi_last=0;
double min_coord_change = 0.00001;
std::string end_button_sym, collect_button_sym;
int end_button_num = 0, collect_button_num = 0;

void joy_CB(const sensor_msgs::Joy joy_msg)
{
	if(joy_msg.buttons[collect_button_num]==1)
	//if(joy_msg.buttons[collect_button_num]==1)
	{
		collect_request = true;

	}	
	if(joy_msg.buttons[end_button_num]==1)
	//if(joy_msg.buttons[end_button_num]==1)
	{
		ROS_INFO("end request registered");
		continue_collection = false;
	}
}

void filtered_gps_CB(const sensor_msgs::NavSatFix gps_msg)
{
		lati_point = gps_msg.latitude;
		longi_point = gps_msg.longitude;
}

int main(int argc, char** argv)
{
	//Initialize variables
		int numWaypoints = 0;
		std::string path_local;

    // Initialize node and time
		ros::init(argc, argv, "collect_gps_waypoints"); //initiate node called collect_gps_waypoints
		ros::NodeHandle n;
		ros::Time::init();
		ros::Rate rate(1);

	// Get button numbers to collect waypoints and end collection
		ros::param::get("collect_button_num", collect_button_num);
		ros::param::get("end_button_num", end_button_num);

    //Initiate subscribers
		ros::Subscriber sub_joy = n.subscribe("/joy_teleop/joy", 100, joy_CB);
		ros::Subscriber sub_gps = n.subscribe("/gps/filtered", 100, filtered_gps_CB);
		ROS_INFO("Initiated collect_gps_waypoints node");

    //Read file path and create/open file
    	ros::param::get("coordinates_file", path_local);
		std::string path_abs =  ros::package::getPath("outdoor_waypoint_nav") + path_local;	
		std::ofstream coordFile (path_abs.c_str());
		ROS_INFO("Saving coordinates to: %s", path_abs.c_str());
		
	// Give instructions:
		ros::param::get("collect_button_sym", collect_button_sym);
		ros::param::get("end_button_sym", end_button_sym);
		ROS_INFO("Press %s button to collect and store waypoint.", collect_button_sym.c_str());
		ROS_INFO("Press %s button to end waypoint collection.", end_button_sym.c_str());
		std::cout << std::endl;

	if(coordFile.is_open())
	{
		while(continue_collection)
		{
			ros::spinOnce();
			if(collect_request == true)
			{
				// Check that there was sufficient change in position between points
				std::cout << lati_point - lati_last << std::endl;
				std::cout << longi_point - longi_last << std::endl;
				if(lati_point - lati_last > min_coord_change | longi_point - longi_last > min_coord_change)
				{
					//write waypoint
					ROS_INFO("You have collected another waypoint!");
					ROS_INFO("Press %s button to collect and store another waypoint.", collect_button_sym.c_str());
					ROS_INFO("Press %s button to end waypoint collection.", end_button_sym.c_str());
					std::cout << std::endl;
					numWaypoints++;
					lati_last = lati_point;
					longi_last = longi_point;
				}
				else
				{//do not write waypoint
				}	
				collect_request = false; //reset
			}
			else{}
			rate.sleep();
		}
		coordFile.close();
	}
	else
	{
		ROS_ERROR("Unable to open file.");
		ROS_INFO("Exiting..");
	}

	ROS_INFO("Closed waypoint file, you have collected %d waypoints", numWaypoints);
	ROS_INFO("Ending node...");
	ros::shutdown();
	return 0;
}

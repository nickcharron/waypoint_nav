#include <ros/ros.h>
#include <ros/package.h>
#include <utility>
#include <fstream>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <math.h>



bool collect_request;
bool continue_collection = true;
double lati_point=0, longi_point=0, lati_last=0, longi_last=0;
double min_coord_change = 10 * pow(10,-6);
std::string end_button_sym, collect_button_sym;
int end_button_num = 0, collect_button_num = 0;

void joy_CB(const sensor_msgs::Joy joy_msg)
{
	if(joy_msg.buttons[collect_button_num]==1)
	{
		collect_request = true;
	}
	else
	{
		collect_request = false;
	}

	if(joy_msg.buttons[end_button_num]==1)
	{
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
		ros::Time time_last;
		ros::Time time_current;
		ros::Duration duration_min(1);

	// Get button numbers to collect waypoints and end collection
		ros::param::get("/outdoor_waypoint_nav/collect_button_num", collect_button_num);
		ros::param::get("/outdoor_waypoint_nav/end_button_num", end_button_num);

    //Initiate subscribers
		ros::Subscriber sub_joy = n.subscribe("/joy_teleop/joy", 100, joy_CB);
		ros::Subscriber sub_gps = n.subscribe("/outdoor_waypoint_nav/gps/filtered", 100, filtered_gps_CB);
		ROS_INFO("Initiated collect_gps_waypoints node");

	// Initiate publisher to send end of node message
		ros::Publisher pubCollectionNodeEnded = n.advertise<std_msgs::Bool>("/outdoor_waypoint_nav/collection_status",100);

    //Read file path and create/open file
    	ros::param::get("/outdoor_waypoint_nav/coordinates_file", path_local);
		std::string path_abs =  ros::package::getPath("outdoor_waypoint_nav") + path_local;	
		std::ofstream coordFile (path_abs.c_str());
		ROS_INFO("Saving coordinates to: %s", path_abs.c_str());
		
	// Give instructions:
		ros::param::get("/outdoor_waypoint_nav/collect_button_sym", collect_button_sym);
		ros::param::get("/outdoor_waypoint_nav/end_button_sym", end_button_sym);
		std::cout << std::endl;
		std::cout << "Press " << collect_button_sym.c_str() << " button to collect and store waypoint." << std::endl;
		std::cout << "Press " << end_button_sym.c_str() << " button to end waypoint collection." << std::endl;
		std::cout << std::endl;

	if(coordFile.is_open())
	{
		while(continue_collection)
		{
			ros::spinOnce();
			time_current = ros::Time::now();
			if((collect_request == true) && (time_current - time_last > duration_min))
			{	
				// Check that there was sufficient change in position between points
				// This makes the move_base navigation smoother and stops points from being collected twice
				double difference_lat = abs((lati_point - lati_last)*pow(10,6))*pow(10,-6);
				double difference_long = abs((longi_point - longi_last)*pow(10,6))*pow(10,-6);

				if( (difference_lat > min_coord_change) || (difference_long > min_coord_change))
				{
					//write waypoint
					ROS_INFO("You have collected another waypoint!");
					ROS_INFO("Press %s button to collect and store another waypoint.", collect_button_sym.c_str());
					ROS_INFO("Press %s button to end waypoint collection.", end_button_sym.c_str());
					std::cout << std::endl;
					numWaypoints++;
					coordFile << std::fixed << std::setprecision(8) << lati_point << " " << longi_point << std::endl;
					lati_last = lati_point;
					longi_last = longi_point;
				}

				else
				{//do not write waypoint
					ROS_WARN("Waypoint not saved, you have not moved enough");
					ROS_WARN("New Latitude: %f   Last Latitude: %f \n", lati_point, lati_last );
					ROS_WARN("New Longitude: %f   Last Longitude: %f \n", longi_point, longi_last );
				}
				time_last = time_current;
			}
			else{}
			ros::spinOnce();
		}
	
		coordFile.close();
		ROS_INFO("End request registered.");
	}
	else
	{
		ROS_ERROR("Unable to open file.");
		ROS_INFO("Exiting..");
	}

	ROS_INFO("Closed waypoint file, you have collected %d waypoints.", numWaypoints);
	ROS_INFO("Ending node...");

	// Notify joy_launch_control that calibration is complete
    std_msgs::Bool node_ended;
    node_ended.data = true;
    pubCollectionNodeEnded.publish(node_ended);

	ros::shutdown();
	return 0;
}

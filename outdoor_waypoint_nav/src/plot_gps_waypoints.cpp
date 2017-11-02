#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <utility>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <math.h>

#include <robot_localization/navsat_conversions.h>
#include <tf/transform_listener.h>

double latiPoint_raw = 0, longiPoint_raw = 0, latiPoint_filtered = 0, longiPoint_filtered = 0;
double utmX_raw = 0, utmY_raw = 0, utmX_filtered = 0, utmY_filtered = 0;
double collection_time;
std::string utmZone, path_local_filtered, path_local_raw, path_abs_filtered, path_abs_raw;
bool collect_request = false;
bool continue_collection = true;
std::string end_button_sym, collect_button_sym;
int end_button_num = 0, collect_button_num = 0, numWaypoints= 0, numWaypoint_clusters= 0, numPoints;

void raw_gps_CB(const sensor_msgs::NavSatFix gps_msg)
{
		latiPoint_raw = gps_msg.latitude;
		longiPoint_raw = gps_msg.longitude;
}

void filtered_gps_CB(const sensor_msgs::NavSatFix gps_msg)
{
		latiPoint_filtered = gps_msg.latitude;
		longiPoint_filtered = gps_msg.longitude;
}

void joy_CB(const sensor_msgs::Joy joy_msg)
{
	if(joy_msg.buttons[collect_button_num]==1)
	{
		collect_request = true;

	}	
	if(joy_msg.buttons[end_button_num]==1)
	{
		ROS_INFO("end request registered");
		continue_collection = false;
	}
}


int main(int argc, char** argv)
{
     // Initialize node and time
		ros::init(argc, argv, "plot_gps_waypoints"); //initiate node called plot_gps_waypoints
		ros::NodeHandle n;
	
    // Get params
		ros::param::get("/outdoor_waypoint_nav/collect_button_num", collect_button_num);
		ros::param::get("/outdoor_waypoint_nav/end_button_num", end_button_num);
    	ros::param::get("/outdoor_waypoint_nav/filtered_coordinates_file", path_local_filtered);
    	ros::param::get("/outdoor_waypoint_nav/raw_coordinates_file", path_local_raw);
		ros::param::get("/outdoor_waypoint_nav/collect_button_sym", collect_button_sym);
		ros::param::get("/outdoor_waypoint_nav/end_button_sym", end_button_sym);
		ros::param::get("/outdoor_waypoint_nav/num_points", numPoints);
		ros::param::get("/outdoor_waypoint_nav/collection_time", collection_time);

    // Initialize time and set rates
		ros::Time::init();
        ros::Rate rate1(1);
		
     //Subscribe to topics
        ros::Subscriber sub_gps_raw = n.subscribe("/navsat/fix", 100, raw_gps_CB);
        ros::Subscriber sub_gps_filtered = n.subscribe("/outdoor_waypoint_nav/gps/filtered", 100, filtered_gps_CB);
		ros::Subscriber sub_joy = n.subscribe("/joy_teleop/joy", 100, joy_CB);


   //Read file path and create/open file
		std::string path_abs_raw =  ros::package::getPath("outdoor_waypoint_nav") + path_local_raw;	
        std::string path_abs_filtered =  ros::package::getPath("outdoor_waypoint_nav") + path_local_filtered;
		std::ofstream coordFile_raw (path_abs_raw.c_str());
        std::ofstream coordFile_filtered (path_abs_filtered.c_str());

		ROS_INFO("Saving raw coordinates to: %s", path_abs_raw.c_str());
        ROS_INFO("Saving filtered coordinates to: %s", path_abs_filtered.c_str());
		
	// Give instructions:
		ROS_INFO("Press %s button to collect and store waypoint.", collect_button_sym.c_str());
		ROS_INFO("Press %s button to end waypoint collection.", end_button_sym.c_str());
		std::cout << std::endl;

		if(coordFile_raw.is_open() && coordFile_filtered.is_open())
		{
	        while(continue_collection)
			{
				ros::spinOnce();
				if(collect_request == true)
				{

					//write waypoint
					std::cout << std::endl;
	                ROS_INFO("Collecting new set of waypoints...");
					numWaypoint_clusters++;

	                // collect one filtered point            
	                RobotLocalization::NavsatConversions::LLtoUTM(latiPoint_filtered, longiPoint_filtered, utmY_filtered, utmX_filtered, utmZone);
	                coordFile_filtered << std::fixed << std::setprecision(8) << utmX_filtered << " " << utmY_filtered << std::endl;
					
	                // collect 30 raw points at 5 hz
	                for(int i=0; i<numPoints; i++)
	                {
	                    ros::spinOnce();
	                    RobotLocalization::NavsatConversions::LLtoUTM(latiPoint_raw, longiPoint_raw, utmY_raw, utmX_raw, utmZone);
	                    coordFile_raw << std::fixed << std::setprecision(8) << utmX_raw << " " << utmY_raw << std::endl;
						numWaypoints++;
						ROS_INFO("Collected GPS Point Number %d.", numWaypoints);
	                    ros::Duration(collection_time).sleep(); 
	                }
					
					ROS_INFO("You have collected another waypoint cluster!");
					ROS_INFO("Press %s button to collect and store another waypoint.", collect_button_sym.c_str());
					ROS_INFO("Press %s button to end waypoint collection.", end_button_sym.c_str());
					collect_request = false; //reset
				}
				else{}
			rate1.sleep();
			}
		}
		else
		{
			ROS_ERROR("Unable to open file.");
			ROS_INFO("Exiting..");
		}
		
		ROS_INFO("Closed waypoint files, you have collected %d waypoint clusters and %d waypoints", numWaypoint_clusters, numWaypoints);
		coordFile_raw.close();
        coordFile_filtered.close();
		ROS_INFO("Ending node...");
		ros::shutdown();

        return 0;
}

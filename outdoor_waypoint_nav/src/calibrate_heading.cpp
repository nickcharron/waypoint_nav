#include <ros/ros.h>
#include <ros/package.h>
#include <utility>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <fstream>
#include <iostream>

#include <math.h>

// Init variables
float y_pos, x_pos, x_vel, x_vel_time, frequency, delay, yaw_offset, magnetic_declination_radians;
bool zero_altitude, broadcast_utm_transform, publish_filtered_gps, use_odometry_yaw, wait_for_datum;


void getParams()
{
    ros::param::get("/outdoor_waypoint_nav/x_vel", x_vel);
    ros::param::get("/outdoor_waypoint_nav/x_vel_time", x_vel_time);
    ros::param::get("/outdoor_waypoint_nav/navsat_transform/frequency", frequency);
    ros::param::get("/outdoor_waypoint_nav/navsat_transform/delay", delay);
    ros::param::get("/outdoor_waypoint_nav/navsat_transform/magnetic_declination_radians", magnetic_declination_radians);
    ros::param::get("/outdoor_waypoint_nav/navsat_transform/yaw_offset", yaw_offset);
    ros::param::get("/outdoor_waypoint_nav/navsat_transform/zero_altitude", zero_altitude);
    ros::param::get("/outdoor_waypoint_nav/navsat_transform/broadcast_utm_transform", broadcast_utm_transform);
    ros::param::get("/outdoor_waypoint_nav/navsat_transform/publish_filtered_gps", publish_filtered_gps);
    ros::param::get("/outdoor_waypoint_nav/navsat_transform/use_odometry_yaw", use_odometry_yaw);
    ros::param::get("/outdoor_waypoint_nav/navsat_transform/wait_for_datum", wait_for_datum);
}

void writeParams(std::string path_to_param_file, double heading_err)
{
    // Open file
    std::ofstream paramsFile (path_to_param_file.c_str());
    
    // Write to file
        paramsFile << "navsat_transform:" << std::endl;
        paramsFile << std::fixed << std::setprecision(0) << "  frequency: " << frequency << std::endl;
        paramsFile << std::fixed << std::setprecision(1) << "  delay: " << delay << std::endl;
    
        // Adding heading error to magnetic declination parameter to correct initial poor estimate
        paramsFile << std::fixed << std::setprecision(5) << "  magnetic_declination_radians: " << (magnetic_declination_radians + heading_err)<< std::endl;
        paramsFile << std::fixed << std::setprecision(5) << "  yaw_offset: " << yaw_offset << std::endl;
        paramsFile << "  zero_altitude: " << std::boolalpha << zero_altitude << std::endl;
        paramsFile << "  broadcast_utm_transform: " << std::boolalpha << broadcast_utm_transform << std::endl;
        paramsFile << "  publish_filtered_gps: " << std::boolalpha << publish_filtered_gps << std::endl;
        paramsFile << "  use_odometry_yaw: " << std::boolalpha << use_odometry_yaw << std::endl;
        paramsFile << "  wait_for_datum: " << std::boolalpha << wait_for_datum << std::endl;

    // Close file
    paramsFile.close();
}

void filtered_odom_CB(const nav_msgs::Odometry odom_msgs)
{
		y_pos = odom_msgs.pose.pose.position.y;
		x_pos = odom_msgs.pose.pose.position.x;
}

int main(int argc, char **argv)
{
    //Initiate node and set hangle
    ros::init(argc, argv, "calibrate_heading");
    ros::NodeHandle n;
    ROS_INFO("Initiated calibration node");

    // Initialise publishers and subscribers
    ros::Subscriber sub_odom = n.subscribe("/outdoor_waypoint_nav/odometry/filtered_map", 100, filtered_odom_CB);
    ros::Publisher pubVel = n.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel",100);
    ros::Publisher pubCalibrationNodeEnded = n.advertise<std_msgs::Bool>("/outdoor_waypoint_nav/calibrate_status",100);

    // Get parameters from parameer server
    getParams();
    ROS_WARN("PLEASE ENSURE YOU HAVE MIN. %.1f m OF CLEAR SPACE IN FRONT OF YOUR ROBOT FOR CALIBRATION",(x_vel*x_vel_time));    

    // set publish rate and calculate no. of messages to count
    int pubRate = 10;
    int numVelMsgs = x_vel_time * pubRate;
    ros::Rate rate(pubRate);

    // Create forward velocity commmands and publish
    geometry_msgs::Twist velmsg;
    velmsg.linear.x= x_vel;
    velmsg.angular.z=0;
    for(int i=0; i< numVelMsgs; i++)
    {
        pubVel.publish(velmsg);
        rate.sleep();
    }
    ros::Duration(2).sleep(); // Pause for 2 seconds to prevent quick forwards and backwards movement

    // Read y value in filtered odometry and determine correction to heading
    ros::spinOnce();
    double heading_error = atan2(y_pos, x_pos);
    ROS_INFO("Detected heading error of: %.1f Degrees", 180/M_PI*(heading_error));

    //write params file
    std::string path =  ros::package::getPath("outdoor_waypoint_nav") + "/params/navsat_params.yaml";
    ROS_INFO("Writing calibration results to file...");
    writeParams(path, heading_error);
    ROS_INFO("Wrote to param file: ");
    std::cout << path.c_str() << std::endl;       

    // Create backward commmands and publish
    ROS_INFO("Returning to start...");
    velmsg.linear.x= -1*x_vel;
    velmsg.angular.z=0;
    for(int i=0; i< numVelMsgs; i++)
    {
        pubVel.publish(velmsg);
        rate.sleep();
    }

    ROS_INFO("Heading Calibration Complete");

    // Notify joy_launch_control that collection is complete
        std_msgs::Bool node_ended;
        node_ended.data = true;
        pubCalibrationNodeEnded.publish(node_ended);
    
    ROS_INFO("Ending Node...");
    ROS_WARN("PLEASE RESTART YOUR EKF NODES TO APPLY NEW CALIBRATION PARAMETERS.");
	ros::shutdown();
	return 0;
}

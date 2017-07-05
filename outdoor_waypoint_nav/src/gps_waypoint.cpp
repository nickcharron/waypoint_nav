#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <math.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; //create a type definition for a client called MoveBaseClient
int waypointCount = 0;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gps_waypoint"); //initiate node called gps_waypoint
	ROS_INFO("Initiated gps_waypoint node");
	MoveBaseClient ac("move_base", true); //construct an action client that we use to
					      //communication with the action named move_base.
					      //Setting true is telling the constructor to start ros::spin()

        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0)))
		{
          ROS_INFO("Waiting for the move_base action server to come up");
        }

	move_base_msgs::MoveBaseGoal goal; //initiate a move_base_msg called goal

	tf::TransformListener listener; //create transformlistener object called listener

	//Get Longitude and Latitude goals from text file
		
		//Initialize variables
		std::vector<std::pair<double,double> > waypointVect;
		double lati=0, longi=0;
		int count = 0;
		double numWaypoints = 0;
	    double latiGoal, longiGoal;
		double utm_x = 0, utm_y = 0;
		float x_prev = 0, y_prev = 0;	//for determining heading goal
		std::string utm_zone;
		std::string path_local;

		//Count number of waypoints and check that the file opens
		
		ros::param::get("coordinates_file", path_local);
    	std::string path_abs =  ros::package::getPath("outdoor_waypoint_nav") + path_local;	
		std::ifstream fileCount (path_abs.c_str());
		if (fileCount.is_open())
		{
	  		while(!fileCount.eof())
	  		{
				fileCount >> lati;
		 		++count;
	  		}
	  		count = count-1;
	  		numWaypoints = count/2;
	  		ROS_INFO("%f GPS waypoints were read", numWaypoints);
      		fileCount.close();
		}
		else
		{ 
	  		std::cout << "Unable to open waypoint file" << std::endl;
	  		ROS_ERROR("Unable to open waypoint file");
      	}
		
		//Reading waypoints from text file and storing in vector
		std::ifstream fileRead (path_abs.c_str());
		for(int i=0; i<numWaypoints; i++)
		{
	  		fileRead >> lati;
			fileRead >> longi;  
	  		waypointVect.push_back(std::make_pair<double,double>(lati,longi));
		}
		fileRead.close();
		
		//Outputting vector
		ROS_INFO("The following GPS Waypoints have been set:\n");
		for(std::vector<std::pair<double, double> >::iterator iterDisp=waypointVect.begin(); iterDisp!=waypointVect.end(); iterDisp++)
		{
			ROS_INFO("%.9g %.9g", iterDisp->first, iterDisp->second);
		}

	// Using Parameter Server to set goal:
		//ros::param::get("latitude_goal", latiGoal);
		//ros::param::get("longitude_goal", longiGoal);

	// Main loop for sending goals from vector
	
	std::vector<std::pair<double, double> >::iterator iter; //init. iterator
	
	for(iter=waypointVect.begin(); iter<waypointVect.end() ; iter++)
	{

		//Setting goal:
		  
		  latiGoal = iter->first;
		  longiGoal = iter->second;
		  ROS_INFO("Received Latitude goal:%f", latiGoal);
	      ROS_INFO("Received longitude goal:%f", longiGoal);

    	//Convert to utm:
		  RobotLocalization::NavsatConversions::LLtoUTM(latiGoal, longiGoal, utm_y, utm_x, utm_zone);
		  ROS_INFO("UTM Cord is %f, %f", utm_x, utm_y);

		//Construct UTM_point and map_point geometry messages
		  geometry_msgs::PointStamped UTM_point;
		  geometry_msgs::PointStamped map_point; 
	
		  UTM_point.header.frame_id = "utm" ;
		  UTM_point.header.stamp = ros::Time(0) ;
		  UTM_point.point.x = utm_x;
		  UTM_point.point.y = utm_y;
		  UTM_point.point.z = 0;


		//Transform UTM to odom frame
		  bool notDone = true;
		  ros::Time time_now = ros::Time::now();
		  while(notDone)
		  {
		      try
		      {
		        UTM_point.header.stamp = ros::Time::now() ;
		        listener.waitForTransform("odom", "utm", time_now, ros::Duration(3.0));
		        listener.transformPoint ("odom", UTM_point, map_point);
		        notDone = false;
		      }
		      catch (tf::TransformException &ex)
		      {
		        ROS_WARN("%s",ex.what());
		        ros::Duration(0.01).sleep();
		        //return;
	    	  }
		  }

    	//Send goal to move_base
		  //Specify what frame we want the goal to be published in 
		  goal.target_pose.header.frame_id = "odom"; 
		  goal.target_pose.header.stamp = ros::Time::now();

		  // Specify x and y goal
		  ROS_INFO("Goal in map frame is  %f, %f", map_point.point.x,map_point.point.y);
		  goal.target_pose.pose.position.x = map_point.point.x; //specify x goal
		  goal.target_pose.pose.position.y = map_point.point.y; //specify y goal

		  // Calculate quaternion
		  tf::Matrix3x3 rot_euler;
		  tf::Quaternion rot_quat;

		  float x_curr = map_point.point.x, y_curr = map_point.point.y; // set current coords.
		  float delta_x = x_curr - x_prev, delta_y = y_curr - y_prev;   // change in coords.
		  float yaw_curr = 0, pitch_curr=0, roll_curr=0;
		  yaw_curr = atan2(delta_y, delta_x);
		  x_prev = x_curr;	// reset previous x and y coordinates
		  y_prev = y_curr;

		  // Specify quaternions
		  rot_euler.setEulerYPR(yaw_curr,pitch_curr, roll_curr);
		  rot_euler.getRotation(rot_quat);
		  
		  goal.target_pose.pose.orientation.x = rot_quat.getX();
		  goal.target_pose.pose.orientation.y = rot_quat.getY();		  
		  goal.target_pose.pose.orientation.z = rot_quat.getZ();		  
		  goal.target_pose.pose.orientation.w = rot_quat.getW();

		  // Send Goal
		  ROS_INFO("Sending goal");
		  ac.sendGoal(goal); //push goal to move_base node

		  //Wait for result
		  ac.waitForResult(); //waiting to see if move_base was able to reach goal

		  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		  {
		    ROS_INFO("Husky has reached its goal!");	
		    //switch to next waypoint and repeat
		  }
		  else
		  {
		    ROS_ERROR("Husky was unable to reach its goal. GPS Waypoint unreachable.");
			ROS_INFO("Exiting node...");
			ros::shutdown();
		  }
	} // End for loop iterating through waypoint vector
	 
	 ROS_INFO("Husky has reached all of its goals!");
	 ROS_INFO("Ending node...");
	 ros::shutdown();
     ros::spin();
	return 0;
}

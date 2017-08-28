#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <math.h>

// initialize variables

	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient1; //create a type definition for a client called MoveBaseClient
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient2;

	int count = 0, waypointCount = 0, wait_count = 0;
    double lati=0, longi=0, numWaypoints=0, latiGoal, longiGoal, latiNext, longiNext, x, y, goal_tolerance;
	bool on_second_controller = false, final_point = false;

	std::vector<std::pair<double,double> > waypointVect;
	std::vector<std::pair<double, double> >::iterator iter; //init. iterator
	std::string utm_zone;
	std::string path_local, path_abs;

	geometry_msgs::PointStamped UTM_point, map_point, UTM_next, map_next;
	std_msgs::Int8 controller_num;

int countWaypointsInFile(std::string path_local)
{
   	path_abs =  ros::package::getPath("outdoor_waypoint_nav") + path_local;	
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
		ROS_INFO("%.0f GPS waypoints were read", numWaypoints);
   		fileCount.close();
	}
	else
	{ 
		std::cout << "Unable to open waypoint file" << std::endl;
		ROS_ERROR("Unable to open waypoint file");
 	}
	 return numWaypoints;
}

std::vector<std::pair<double,double> > getWaypoints(std::string path_local)
{
 	path_abs =  ros::package::getPath("outdoor_waypoint_nav") + path_local;	
	std::ifstream fileRead (path_abs.c_str());
	for(int i=0; i<numWaypoints; i++)
	{
  		fileRead >> lati;
		fileRead >> longi;  
  		waypointVect.push_back(std::make_pair<double,double>(lati,longi));
	}
	fileRead.close();
	
	//Outputting vector
	ROS_INFO("The following GPS Waypoints have been set:");
	for(std::vector<std::pair<double, double> >::iterator iterDisp=waypointVect.begin(); iterDisp!=waypointVect.end(); iterDisp++)
	{
		ROS_INFO("%.9g %.9g", iterDisp->first, iterDisp->second);
	}
	return waypointVect;
}

geometry_msgs::PointStamped latLongtoUTM(double lati_input, double longi_input)
{
		double utm_x = 0, utm_y = 0;
		geometry_msgs::PointStamped UTM_point_output;

		//convert lat/long to utm
		  RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);

		//Construct UTM_point and map_point geometry messages
		  UTM_point_output.header.frame_id = "utm" ;
		  UTM_point_output.header.stamp = ros::Time(0) ;
		  UTM_point_output.point.x = utm_x;
		  UTM_point_output.point.y = utm_y;
		  UTM_point_output.point.z = 0;

		  return UTM_point_output;
}

geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input)
{
	geometry_msgs::PointStamped map_point_output;
	bool notDone = true;
	tf::TransformListener listener; //create transformlistener object called listener
	ros::Time time_now = ros::Time::now();
	while(notDone)
	{
	   try
	   {
	       UTM_point.header.stamp = ros::Time::now() ;
	       listener.waitForTransform("odom", "utm", time_now, ros::Duration(3.0));
	       listener.transformPoint ("odom", UTM_input, map_point_output);
	       notDone = false;
	   }
	   catch (tf::TransformException &ex)
	   {
		   ROS_WARN("%s",ex.what());
	       ros::Duration(0.01).sleep();
	       //return;
	   }
	}
	return map_point_output;
}

move_base_msgs::MoveBaseGoal buildGoal(geometry_msgs::PointStamped map_point)
{
	move_base_msgs::MoveBaseGoal goal;

	//Specify what frame we want the goal to be published in 
	goal.target_pose.header.frame_id = "odom"; 
	goal.target_pose.header.stamp = ros::Time::now();

	// Specify x and y goal
	goal.target_pose.pose.position.x = map_point.point.x; //specify x goal
	goal.target_pose.pose.position.y = map_point.point.y; //specify y goal
	goal.target_pose.pose.orientation.w = 1.0;	// don't care about heading because we aren't actually acheiving our goal
		
	return goal;
}

void odometry_CB(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	x = odom_msg->pose.pose.position.x;
	y = odom_msg->pose.pose.position.y;
}

void waitToReachGoal(double map_x, double map_y, double goal_tolerance)
{
	while(sqrt((map_x-x)*(map_x-x)+(map_y-y)*(map_y-y)) > goal_tolerance) 
	{
		// do nothing
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gps_waypoint_1"); //initiate node called gps_waypoint
	ros::NodeHandle n1;
	NodeConfiguration configuration = NodeConfiguration.createDefault();
	configuration.setHost(host);
	configuration.setMasterUri(uri);
	NameResolver res = NameResolver.createFromString("/controller_1");
	configuration.setParentResolver(res);

	ROS_INFO("Initiated gps_waypoint node");
	MoveBaseClient1 ac1("move_base", true);


	MoveBaseClient2 ac2("move_base", true);  
		//construct an action client that we use to communication with the action named move_base1.
		//Setting true is telling the constructor to start ros::spin()

	// Initiate publisher to send end of node message and publisher to say which node is publishing the proper vel commands
		ros::Publisher pubWaypointNodeEnded = n.advertise<std_msgs::Bool>("outdoor_waypoint_nav/waypoint_following_status",100);
		ros::Publisher pub_controller_num = n.advertise<std_msgs::Int8>("/outdoor_waypoint_nav/controller_in_use", 100);

	// Initiate subscriber to subscribe to filtered odometery
		ros::Subscriber sub_odom = n.subscribe("odometry/filtered_map", 1000, odometry_CB);

    //wait for the first action server to come up
   		while(!ac1.waitForServer(ros::Duration(5.0)))
		{
			wait_count++;
			if(wait_count > 3)
			{
				ROS_ERROR("move_base action server did not come up, killing gps_waypoint node...");
				// Notify joy_launch_control that waypoint following is complete
     			std_msgs::Bool node_ended;
    			node_ended.data = true;
     			pubWaypointNodeEnded.publish(node_ended);
				ros::shutdown();
			}
	      	ROS_INFO("Waiting for the move_base action server to come up");
        }

	//Get Longitude and Latitude goals from text file

		//Count number of waypoints 
		ros::param::get("coordinates_file", path_local);
		numWaypoints = countWaypointsInFile(path_local);

		//Reading waypoints from text file and output results
		waypointVect = getWaypoints(path_local);

	// Iterate through vector of waypoints for setting goals
	for(iter=waypointVect.begin(); iter<waypointVect.end() ; iter++)
	{		 
		//Setting goal:
		  latiGoal = iter->first;
		  longiGoal = iter->second;
		  ROS_INFO("Received Latitude goal:%.8f", latiGoal);
	      ROS_INFO("Received longitude goal:%.8f", longiGoal);	

		  //set next goal if not at last waypoint
		  if(iter < (waypointVect.end()-1))
		  {
			  iter++;
			  latiNext = iter->first;
 			  longiNext = iter->second;
			  //iter--;  - we want to skip every other goal
		  }
		  else
		  {
			  final_point = true;
		  }

    	//Convert lat/long to utm:
		  UTM_point = latLongtoUTM(latiGoal, longiGoal);
		  UTM_next = latLongtoUTM(latiNext, longiNext);

		//Transform UTM to map point in odom frame
		  map_point = UTMtoMapPoint(UTM_point);
		  map_next = UTMtoMapPoint(UTM_next);

    	//Build goal to send to move_base
		  move_base_msgs::MoveBaseGoal goal = buildGoal(map_point); //initiate a move_base_msg called goal

		//Build next goal if not on last waypoint
		  move_base_msgs::MoveBaseGoal goal_next = buildGoal(map_next); //initiate a move_base_msg called goal_next

		// Publish which node is sending the correct velocity commands
		  if(on_second_controller == false)
		  {
			  controller_num.data = 1;
		  }
		  else
		  {
			  controller_num.data = 2;
		  }
		  pub_controller_num.publish(controller_num);

		//Send Goals
		  ros::param::get("goalTolerance", goal_tolerance);
		  ROS_INFO("Sending goal");
		  if(on_second_controller == true)
		  {
			  // wait for robot to get close enough to goal
    			waitToReachGoal(map_point.point.x, map_point.point.y, goal_tolerance);
		  }
		  
		  ac1.sendGoal(goal); //push current goal to move_base node
		  on_second_controller = false; //want to switch controllers for next waypoint			  

			// Wait for robot to get close enough to its goal
				waitToReachGoal(map_next.point.x, map_next.point.y, goal_tolerance);

		// change to other controller
		  ac2.sendGoal(goal); // push current goal to move_base
	      on_second_controller = false; //want to swtich controllers for next waypoint
	} // End for loop iterating through waypoint vector
	 
	 ROS_INFO("Husky has reached all of its goals!!!\n");
	 ROS_INFO("Ending node...");

	 // Notify joy_launch_control that waypoint following is complete
     std_msgs::Bool node_ended;
     node_ended.data = true;
     pubWaypointNodeEnded.publish(node_ended);

	 ros::shutdown();
     ros::spin();
	return 0;
}

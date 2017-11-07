#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; //create a type definition for a client called MoveBaseClient

int count = 0, waypointCount = 0, wait_count = 0;
double numWaypoints=0, latiGoal, longiGoal, latiNext, longiNext, x, y, goal_tolerance;
bool end_on_controller_1=false;

std::vector<std::pair<double,double> > waypointVect;
std::vector<std::pair<double, double> >::iterator iter; //init. iterator
std::string utm_zone;
std::string path_local, path_abs;

geometry_msgs::PointStamped UTM_point, map_point, UTM_next, map_next;
std_msgs::Bool controller_1_done, controller_2_done;

int countWaypointsInFile(std::string path_local)
{
    path_abs = ros::package::getPath("outdoor_waypoint_nav") + path_local;
    std::ifstream fileCount(path_abs.c_str());
    if(fileCount.is_open())
    {
        double lati=0;

        while(!fileCount.eof())
        {
            fileCount >> lati;
            ++count;
        }

        count = count - 1;
        numWaypoints = count / 2;
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

std::vector <std::pair<double, double>> getWaypoints(std::string path_local)
{
    double lati=0, longi=0;
    path_abs = ros::package::getPath("outdoor_waypoint_nav") + path_local;
    std::ifstream fileRead(path_abs.c_str());
    for(int i = 0; i < numWaypoints; i++)
    {
        fileRead >> lati;
        fileRead >> longi;
        waypointVect.push_back(std::make_pair(lati, longi));
    }
    fileRead.close();

    //Outputting vector
    ROS_INFO("The following GPS Waypoints have been set:");
    for(std::vector < std::pair < double, double > > ::iterator iterDisp = waypointVect.begin(); iterDisp != waypointVect.end();
    iterDisp++)
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
    UTM_point_output.header.frame_id = "utm";
    UTM_point_output.header.stamp = ros::Time(0);
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
            UTM_point.header.stamp = ros::Time::now();
            listener.waitForTransform("odom", "utm", time_now, ros::Duration(3.0));
            listener.transformPoint("odom", UTM_input, map_point_output);
            notDone = false;
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
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
    goal.target_pose.pose.orientation.w = 1.0;    // don't care about heading because we aren't actually acheiving our goal

    return goal;
}

void odometry_CB(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    x = odom_msg->pose.pose.position.x;
    y = odom_msg->pose.pose.position.y;
}

void controller_2_CB(const std_msgs::Bool::ConstPtr& controller_2_done_msg)
{
    if(controller_2_done_msg->data == true)
    {
        controller_2_done.data = true;
    }
    else
    {
        controller_2_done.data = false;
    }
}

void waitToReachGoal(double map_x, double map_y, double goal_tolerance);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_waypoint_1"); //initiate node called gps_waypoint
    ros::NodeHandle n;
    ROS_INFO("Initiated gps_waypoint node 1");
    MoveBaseClient ac1("/controller_1/move_base", true);
    //construct an action client that we use to communication with the action named move_base.
    //Setting true is telling the constructor to start ros::spin()

    // Initiate publisher to send end of node message and publisher to say which node is publishing the proper vel commands
    ros::Publisher pubWaypointNodeEnded = n.advertise<std_msgs::Bool>("/outdoor_waypoint_nav/waypoint_following_status", 1000);
    ros::Publisher pub_controller_1_done = n.advertise<std_msgs::Bool>("/controller_1/controller_1_done", 1000);

    // Initiate subscriber to subscribe to filtered odometery
    ros::Subscriber sub_odom = n.subscribe("/outdoor_waypoint_nav/odometry/filtered_map", 1000, odometry_CB);
    ros::Subscriber sub_controller_2_status = n.subscribe("/controller_2/controller_2_done", 1000, controller_2_CB);

    controller_1_done.data = false;
    controller_2_done.data = false;
    ros::param::get("/outdoor_waypoint_nav/goalTolerance", goal_tolerance);

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
        ROS_INFO("Waiting for the move_base action server 1 to come up");
    }

    //Get Longitude and Latitude goals from text file

    //Count number of waypoints
    ros::param::get("/outdoor_waypoint_nav/coordinates_file", path_local);
    numWaypoints = countWaypointsInFile(path_local);

    //Reading waypoints from text file and output results
    waypointVect = getWaypoints(path_local);

    // Iterate through vector of waypoints for setting goals
    for(iter = waypointVect.begin(); iter < waypointVect.end(); iter++)
    {
        //Setting goal:
        latiGoal = iter->first;
        longiGoal = iter->second;
        ROS_INFO("Controller 1: Received Latitude goal:%.8f", latiGoal);
        ROS_INFO("Controller 1: Received longitude goal:%.8f", longiGoal);

        if(iter < (waypointVect.end() - 2)) // keep incrementing iter, we want to increment by two each time
        {
            iter++;
        }
        else if(iter
                == (waypointVect.end() - 2)) // this means controller 1 is on its last waypoint point but controller 2 has another to go
        {
            iter++;
            end_on_controller_1 = false;
        }
        else if(iter
                == (waypointVect.end() - 1))  // this means that controller 1 is on THE last waypoint and should shutdown when it's done
        {
            end_on_controller_1 = true;
        }
        else
        {
            ROS_ERROR("Controller 1: Error with waypoint vector iterator.");
            ros::shutdown();
        }

        //Convert lat/long to utm:
        UTM_point = latLongtoUTM(latiGoal, longiGoal);

        //   UTM_next = latLongtoUTM(latiNext, longiNext);

        //Transform UTM to map point in odom frame
        map_point = UTMtoMapPoint(UTM_point);
        //   map_next = UTMtoMapPoint(UTM_next);

        //Build goal to send to move_base
        move_base_msgs::MoveBaseGoal goal = buildGoal(map_point); //initiate a move_base_msg called goal

        // wait for controller 2 to give signal (unless at first waypoint)
        if(iter == waypointVect.begin() + 1)
        {
            // keep going
        }
        else
        {
            while(controller_2_done.data == false)
            {
                ros::spinOnce();
                // wait
            }
            ROS_INFO("Controller 1: Received start signal from Controller 2");
            controller_2_done.data = false;
        }

        //Send Goals
        ROS_INFO("Controller 1: Sending goal");
        ac1.sendGoal(goal); //push current goal to move_base node

        if(end_on_controller_1 == false) // if this is the last point then we want to skip this step
        {
            waitToReachGoal(map_point.point.x, map_point.point.y, goal_tolerance);
            //   ROS_INFO("Controller 1: Sending start command to controller 2");
            controller_1_done.data = true; // once done waiting, publish that this controller is done, and to switch to the next
            pub_controller_1_done.publish(controller_1_done);
            controller_1_done.data = false; //reset
        }
        else //if on last waypoint, wait for it to acheive its goal
        {
            // ROS_ERROR("Controller 1: Waiting for result");
            ac1.waitForResult();
            // ROS_ERROR("Controller 1: have result");
            //controller_1_done.data = true; // once done waiting, publish that this controller is done, and to switch to the next
            //pub_controller_1_done.publish(controller_1_done);

            ROS_INFO("Husky has reached all of its goals!!!\n");

            // Notify joy_launch_control that waypoint following is complete
            std_msgs::Bool node_ended;
            node_ended.data = true;
            pubWaypointNodeEnded.publish(node_ended);

            ROS_INFO("Ending controller 1 node...");
            ros::shutdown();
        }
    } // End for loop iterating through waypoint vector

    ROS_INFO("Ending controller 1 node...");
    ros::shutdown();

    //ros::spin();
    return 0;
}

void waitToReachGoal(double map_x, double map_y, double goal_tolerance)
{
    ros::Time time_last = ros::Time::now();
    ros::Time time_last_distance_check = ros::Time::now();
    double last_distance_to_goal = sqrt((map_x - x) * (map_x - x) + (map_y - y) * (map_y - y)), current_distance_to_goal;
    bool is_distance_changing = true;

    ROS_INFO("Controller 1: Waiting for robot to approach goal...");
    // ROS_INFO("Controller 1: Goal Tolerance: %.1f m", goal_tolerance);
    while(sqrt((map_x - x) * (map_x - x) + (map_y - y) * (map_y - y)) > goal_tolerance)
    {
        current_distance_to_goal = sqrt((map_x - x) * (map_x - x) + (map_y - y) * (map_y - y));

        if((ros::Time::now() - time_last) > ros::Duration(1))
        {
            ROS_INFO("Controller 1: Distance to Goal: %.2f", current_distance_to_goal);
            time_last = ros::Time::now();
        }
        if((ros::Time::now() - time_last_distance_check) > ros::Duration(7))
        {
            // check that it has moved enough
            if(abs(current_distance_to_goal - last_distance_to_goal) < 0.1)
            {
                is_distance_changing = false;
            }
            time_last_distance_check = ros::Time::now();
            last_distance_to_goal = current_distance_to_goal;
        }
        ros::spinOnce();
    }
    if(is_distance_changing == false)
    {
        ROS_WARN("Controller 1: Distance to goal not changing, switching to next goal");
    }
    else
    {
        ROS_INFO("Controller 1: goal tolerance reached, sending start signal to controller 2...");
    }
}
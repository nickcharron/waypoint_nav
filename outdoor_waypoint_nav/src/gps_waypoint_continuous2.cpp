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
bool end_on_controller_1 = false;

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
        //ROS_INFO("%.0f GPS waypoints were read", numWaypoints);
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
    path_abs = ros::package::getPath("outdoor_waypoint_nav") + path_local;
    std::ifstream fileRead(path_abs.c_str());
    double lati=0, longi=0;
    for(int i = 0; i < numWaypoints; i++)
    {
        fileRead >> lati;
        fileRead >> longi;
        waypointVect.push_back(std::make_pair(lati, longi));
    }
    fileRead.close();

    //Outputting vector
    // ROS_INFO("The following GPS Waypoints have been set:");
    // for(std::vector<std::pair<double, double> >::iterator iterDisp=waypointVect.begin(); iterDisp!=waypointVect.end(); iterDisp++)
    // {
    // 	ROS_INFO("%.9g %.9g", iterDisp->first, iterDisp->second);
    // }
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

void controller_1_CB(const std_msgs::Bool::ConstPtr& controller_1_done_msg)
{
    if(controller_1_done_msg->data == true)
    {
        controller_1_done.data = true;
    }
    else
    {
        controller_1_done.data = false;
    }
}

void waitToReachGoal(double map_x, double map_y, double goal_tolerance);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_waypoint_2"); //initiate node called gps_waypoint
    ros::NodeHandle n;
    ROS_INFO("Initiated gps_waypoint node 2");
    MoveBaseClient ac2("/controller_2/move_base", true);
    //construct an action client that we use to communication with the action named move_base1.
    //Setting true is telling the constructor to start ros::spin()

    // Initiate publisher to send end of node message and publisher to say which node is publishing the proper vel commands
    ros::Publisher pubWaypointNodeEnded = n.advertise<std_msgs::Bool>("/outdoor_waypoint_nav/waypoint_following_status", 1000);
    ros::Publisher pub_controller_2_done = n.advertise<std_msgs::Bool>("/controller_2/controller_2_done", 1000);

    // Initiate subscriber to subscribe to filtered odometery
    ros::Subscriber sub_odom = n.subscribe("/outdoor_waypoint_nav/odometry/filtered_map", 1000, odometry_CB);
    ros::Subscriber sub_controller_1_status = n.subscribe("/controller_1/controller_1_done", 1000, controller_1_CB);

    controller_1_done.data = false;
    controller_2_done.data = false;
    ros::param::get("/outdoor_waypoint_nav/goalTolerance", goal_tolerance);

    //wait for the first action server to come up
    while(!ac2.waitForServer(ros::Duration(5.0)))
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
        ROS_INFO("Waiting for the move_base action server 2 to come up");
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

        //set next goal if not at last waypoint
        if(iter < (waypointVect.end()
                   - 2)) // this means that neither controller 1 or controller 2 are on the last waypoint so continue incrementing iter by 2
        {
            iter++;
            latiNext = iter->first;
            longiNext = iter->second;
            ros::Duration(0.5).sleep(); // sleeping for half a second to let controller 1 print to window first
            ROS_INFO("Controller 2: Received Latitude goal:%.8f", latiNext);
            ROS_INFO("Controller 2: Received longitude goal:%.8f", longiNext);
        }
        else if(iter == (waypointVect.end()
                         - 2)) // this means that controller 1 is on the second last point so increment iter once more and shutdown after controller 2 is done
        {
            end_on_controller_1 = false;
            iter++;
            latiNext = iter->first;
            longiNext = iter->second;
            ros::Duration(0.5).sleep(); // sleeping for half a second to let controller 1 print to window first
            ROS_INFO("Controller 2: Received Latitude goal:%.8f", latiNext);
            ROS_INFO("Controller 2: Received longitude goal:%.8f", longiNext);
        }
        else if(iter == (waypointVect.end()
                         - 1)) // this means that controller 1 is on its last waypoint and controller 2 should wait for controller 1 to shutdown
        {
            end_on_controller_1 = true;
        }
        else
        {
            ROS_ERROR("Controller 2: Error with waypoint vector iterator.");
            ros::shutdown();
        }

        if(end_on_controller_1 == false) // continue using controller 2, else wait for controller 1 to end
        {
            //Convert lat/long to utm:
            //   UTM_point = latLongtoUTM(latiGoal, longiGoal);
            UTM_next = latLongtoUTM(latiNext, longiNext);

            //Transform UTM to map point in odom frame
            //   map_point = UTMtoMapPoint(UTM_point);
            map_next = UTMtoMapPoint(UTM_next);

            //Build goal to send to move_base
            move_base_msgs::MoveBaseGoal goal = buildGoal(map_next); // controller 2 goes to next map point

            // wait for controller 1 to give signal to start
            ROS_INFO("Controller 2: Waiting for signal from Controller 1...");
            while(controller_1_done.data == false)
            {
                ros::spinOnce();
                // wait
            }
            ROS_INFO("Controller 2: Received start signal from Controller 1");
            controller_1_done.data = false;

            //Send Goals
            ROS_INFO("Controller 2: Sending goal");
            ac2.sendGoal(goal); //push current goal to move_base node
            waitToReachGoal(map_next.point.x, map_next.point.y, goal_tolerance);
            // ROS_INFO("Controller 2: Sending start command to controller 1.");
            if(iter != (waypointVect.end() - 1))
            {
                controller_2_done.data = true; // once done waiting, publish that this controller is done, and to switch to the next
                pub_controller_2_done.publish(controller_2_done);
                controller_2_done.data = false; //reset
            }
            else
            {
                ac2.waitForResult();
                controller_2_done.data = true; // once done waiting, publish that this controller is done, and to switch to the next
                pub_controller_2_done.publish(controller_2_done);

                ROS_INFO("Husky has reached all of its goals!!!\n");

                // Notify joy_launch_control that waypoint following is complete
                std_msgs::Bool node_ended;
                node_ended.data = true;
                pubWaypointNodeEnded.publish(node_ended);

                ROS_INFO("Ending controller 2 node...");
                ros::shutdown();
            }
        }

    } // End for loop iterating through waypoint vector

    if(end_on_controller_1 == false)
    {

    }

    ROS_INFO("Ending controller 2 node...");
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

    ROS_INFO("Controller 2: Waiting for robot to approach goal...");
    // ROS_INFO("Controller 2: Goal Tolerance: %.1f m", goal_tolerance);
    while(sqrt((map_x - x) * (map_x - x) + (map_y - y) * (map_y - y)) > goal_tolerance && is_distance_changing)
    {
        current_distance_to_goal = sqrt((map_x - x) * (map_x - x) + (map_y - y) * (map_y - y));

        if((ros::Time::now() - time_last) > ros::Duration(1))
        {
            ROS_INFO("Controller 2: Distance to Goal: %.2f", current_distance_to_goal);
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
        ROS_WARN("Controller 2: Distance to goal not changing, switching to next goal");
    }
    else
    {
        ROS_INFO("Controller 2: goal tolerance reached, sending start signal to controller 2...");
    }
}
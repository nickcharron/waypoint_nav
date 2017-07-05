#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include <fstream>


int main(int argc, char **argv)
{
    //Initiate publish_waypoints node
    ros::init(argc, argv, "publish_waypoints");

    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("waypoint_array", 100);

    float num=0;
	bool repeat=true;
    std_msgs::Float32MultiArray array;
	int count = 0;
	double numWaypoints = 0;

	//Clear array
	array.data.clear();

    //Reading array from text file
    std::string path =  ros::package::getPath("outdoor_waypoint_nav") + "/src/points.txt";	

	std::ifstream File (path.c_str());
	if (File.is_open())
	{
	  while(!File.eof())
	  {
		  File >> num;
		  ++count;
	  }
	  count = count-1;
	  numWaypoints = count/2;
	  ROS_INFO("%f GPS waypoints were read", numWaypoints);
      File.close();
	}
	else
	{ 
	  std::cout << "Unable to open waypoint file" << std::endl;
	  ROS_ERROR("Unable to open waypoint file");
      repeat = false;
	}
    
	// Construct MultiArray:
	
	std::ifstream File2 (path.c_str());
	for(int i=0; i<count; i++)
	{
	  File2 >> num;
	  array.data.push_back(num);
	}
	File2.close();
    //std::cout << std::endl << array.data[1];

    while(ros::ok() && repeat)
    {
	  //Publish array
	  pub.publish(array);
    }
	//ros::spin();
    return 0;
}

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pubVel;
void cmd_vel_CB(const geometry_msgs::Twist vel_msg);

int main(int argc, char** argv)
{
    // Initialize node
    ros::init(argc, argv, "safety_node"); //initiate node called safety_node
    ros::NodeHandle n;
    ROS_INFO("Initiated safety_node");
    ros::Rate rate(50);

    // Initiate publisher to remap topic send end of node message
    pubVel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    //Initiate subscriber
    ros::Subscriber sub_cmd_vel = n.subscribe("/cmd_vel_intermediate", 100, cmd_vel_CB);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void cmd_vel_CB(const geometry_msgs::Twist vel_msg)
{
    // Publish msg
    // std::cout << "Safety Node: Publishing velocity command of x=" << vel_msg.linear.x << std::endl;
    // std::cout << "                                        twist=" << vel_msg.angular.z << std::endl;
    pubVel.publish(vel_msg);
}
#!/usr/bin/env python
import roslaunch
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

# Define location of launch files
location_collect = "/home/ncharron/catkin_ws/src/waypoint_nav/outdoor_waypoint_nav/launch/collect_goals_sim.launch"
location_send = "/home/ncharron/catkin_ws/src/waypoint_nav/outdoor_waypoint_nav/launch/send_goals_sim.launch"
location_calibrate = "/home/ncharron/catkin_ws/src/waypoint_nav/outdoor_waypoint_nav/launch/heading_calibration_sim.launch"

# Initialize variables
buttons_array = [0, 0, 0]
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid,[location_collect])
calibrate_complete = False
collect_complete = False
send_complete = False

def joy_CB(joy_msg):
    global start_collect_btn
    global buttons_array 
    buttons_array = [joy_msg.buttons[4],joy_msg.buttons[5],joy_msg.buttons[6]]

def calibrate_status_CB(calibrate_status_msg):
    global calibrate_complete
    calibrate_complete = calibrate_status_msg.data

def launch_subscriber():
    rospy.init_node('joy_launch_control')
    rospy.Subscriber("/joy_teleop/joy",Joy, joy_CB )
    rospy.Subscriber("/outdoor_waypoint_nav/calibrate_status",Bool, calibrate_status_CB )

def print_instructions():
    print("")
    print("Press LB to start waypoint collection")
    print("Press RB to start waypoint following")
    print("Press LT to perform heading calibration")
    print("")

def check_buttons():

    global buttons_array     
    global launch 
    global calibrate_complete
    global collect_complete
    global send_complete

    if buttons_array[0] == 1:
        while buttons_array[0] == 1:
            pass
        rospy.loginfo("Starting collect_goals_sim.launch...")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,[location_collect])
        launch.start()

    elif buttons_array[1] == 1:
        while buttons_array[1] ==1:
            pass
        rospy.loginfo("Starting send_goals_sim.launch...")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [location_send])
        launch.start()

    elif buttons_array[2] == 1:
        while buttons_array[2] ==1:
            pass
        rospy.loginfo("Starting heading_calibration.launch...")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [location_calibrate])
        launch.start()

    if calibrate_complete or collect_complete or send_complete:
        rospy.sleep(2) # Sleep for 2 seconds to allow file to be written in other nodes
        launch.shutdown()
        print_instructions()
        # Reset all parameters
        calibrate_complete = False
        collect_complete = False
        send_complete = False

def main():

    # start node to subscribe to joy messages node end messages 
    launch_subscriber()

    # check buttons and launch the appropriate file
    while not rospy.is_shutdown():
        check_buttons()
    rospy.spin()

if __name__ == '__main__':
    print_instructions()
    print("NOTE: It is recommended to perform one or two heading calibrations")
    print("      each time the robot is starting at a new heading.")
    main()
    
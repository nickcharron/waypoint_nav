#!/usr/bin/env python
import rospy
import roslaunch
import rospkg
import os
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

# Initialize variables

buttons_array = [0, 0, 0, 0, 0]
collect_btn_num = 0
collect_btn_sym = ""
send_btn_num = 0
send_btn_sym = ""
calibrate_btn_num = 0
calibrate_btn_sym = ""
abort_btn_num = 0
abort_btn_sym = ""
sim_enabled = False

location_collect = ""
location_send = ""
location_calibrate = ""
location_safety_node = ""

calibrate_complete = False
collect_complete = False
send_complete = False
velocity_paused = False

def getParameter():
    global collect_btn_num
    global collect_btn_sym
    global send_btn_num
    global send_btn_sym
    global calibrate_btn_num
    global calibrate_btn_sym
    global abort_btn_num
    global abort_btn_sym
    global continue_btn_num
    global continue_btn_sym
    global sim_enabled

    collect_btn_num = rospy.get_param("/outdoor_waypoint_nav/collect_button_num")
    collect_btn_sym = rospy.get_param("/outdoor_waypoint_nav/collect_button_sym")
    send_btn_num = rospy.get_param("/outdoor_waypoint_nav/send_button_num")
    send_btn_sym = rospy.get_param("/outdoor_waypoint_nav/send_button_sym")
    calibrate_btn_num = rospy.get_param("/outdoor_waypoint_nav/calibrate_button_num")
    calibrate_btn_sym = rospy.get_param("/outdoor_waypoint_nav/calibrate_button_sym")
    abort_btn_num = rospy.get_param("/outdoor_waypoint_nav/abort_button_num")
    abort_btn_sym = rospy.get_param("/outdoor_waypoint_nav/abort_button_sym")
    continue_btn_num = rospy.get_param("/outdoor_waypoint_nav/continue_button_num")
    continue_btn_sym = rospy.get_param("/outdoor_waypoint_nav/continue_button_sym")
    
    sim_enabled = rospy.get_param("/outdoor_waypoint_nav/sim_enabled")

def getPaths():
    global location_collect
    global location_send
    global location_calibrate
    global location_safety_node
    rospack = rospkg.RosPack()
    
    # Define location of launch files
    if sim_enabled == True:
        location_collect = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/collect_goals_sim.launch"
        location_send = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/send_goals_sim.launch"
        location_calibrate = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/heading_calibration_sim.launch"
        location_safety_node = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/safety_node.launch"

    elif sim_enabled == False:
        location_collect = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/collect_goals.launch"
        location_send = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/send_goals.launch"
        location_calibrate = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/heading_calibration.launch"
        location_safety_node = rospack.get_path('outdoor_waypoint_nav') + "/launch/include/safety_node.launch"

    else:
        print("ERROR: PLEASE SPECIFY SIM_ENABLED PARAMETER.")

def joy_CB(joy_msg):
    global start_collect_btn
    global buttons_array 
    buttons_array = [joy_msg.buttons[collect_btn_num],joy_msg.buttons[send_btn_num],joy_msg.buttons[calibrate_btn_num], joy_msg.buttons[abort_btn_num], joy_msg.buttons[continue_btn_num]]

def calibrate_status_CB(calibrate_status_msg):
    global calibrate_complete
    calibrate_complete = calibrate_status_msg.data

def collection_status_CB(collection_status_msg):
    global collect_complete
    collect_complete = collection_status_msg.data

def waypoint_following_status_CB(waypoint_following_status_msg):
    global send_complete
    send_complete = waypoint_following_status_msg.data

def launch_subscribers():
    rospy.init_node('joy_launch_control')
    rospy.Subscriber("/joy_teleop/joy",Joy, joy_CB )
    rospy.Subscriber("/outdoor_waypoint_nav/calibrate_status",Bool, calibrate_status_CB )
    rospy.Subscriber("/outdoor_waypoint_nav/collection_status",Bool, collection_status_CB )
    rospy.Subscriber("/outdoor_waypoint_nav/waypoint_following_status",Bool, waypoint_following_status_CB )

def print_instructions():

    print ""
    print "Press %s to start waypoint collection" % collect_btn_sym
    print "Press %s to start waypoint following" % send_btn_sym
    print "Press %s to perform heading calibration" % calibrate_btn_sym
    print "Press %s at ANY TIME to STOP robot motion" % abort_btn_sym
    print ""

def check_buttons():

    global buttons_array     
    global launch 
    global calibrate_complete
    global collect_complete
    global send_complete
    global velocity_paused
    
    # Check abort button
    if buttons_array[3] == 1:
        rospy.logerr("STOP BUTTON SELECTED, blocking velocity commands...")
        os.system("rosnode kill safety_node")
        rospy.sleep(1) # Sleep for 1 second to allow time for node to shutdown
        print ""
        print "Press %s to continue following waypoints" % continue_btn_sym
        print ""
        velocity_paused = True
    
    elif buttons_array[4] == 1 and velocity_paused == True:
        rospy.loginfo("continuing to follow wapoints...")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,[location_safety_node])
        launch.start()
        velocity_paused = False

    # Start collecting goals
    if buttons_array[0] == 1:
        while buttons_array[0] == 1:    # Wait for button to be released
            pass
        rospy.loginfo("Starting collect_goals.launch...")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,[location_collect])
        launch.start()

    # Start sending goals
    elif buttons_array[1] == 1:
        while buttons_array[1] ==1:
            pass
        rospy.loginfo("Starting send_goals.launch...")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [location_send])
        launch.start()

    # Start Heading Calbration
    elif buttons_array[2] == 1:
        while buttons_array[2] ==1:
            pass
        rospy.loginfo("Starting heading_calibration.launch...")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [location_calibrate])
        launch.start()

    # Check if end notice has been published by other nodes
    if (calibrate_complete or collect_complete or send_complete):
        rospy.sleep(2) # Sleep for 2 seconds to allow time for other nodes to shutdown
        launch.shutdown()
        print_instructions()
        # Reset all parameters
        calibrate_complete = False
        collect_complete = False
        send_complete = False

def main():

    # start node to subscribe to joy messages node end messages 
    launch_subscribers()

    # check buttons and launch the appropriate file
    while not rospy.is_shutdown():
        check_buttons()
    rospy.spin()

if __name__ == '__main__':

    getParameter()
    getPaths()

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid,[location_collect])
    print_instructions()

    if sim_enabled == False:
        print("NOTE: It is recommended to perform one or two heading calibrations")
        print("      each time the robot is starting from a new heading.")
    
    main()
    
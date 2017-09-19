# waypoint_nav

This package performs outdoor GPS waypoint navigation. It can navigate while building a map, avoiding obstacles, and can navigate continuously between each goal or stop at each goal. 

This repo is made to run on a Clearpath Husky with IMU, Novatel GPS, and Sick lms111 lidar.

This package uses a combination of the following packages:
	- ekf_localization to fuse odometry data with IMU and GPS data
	- navsat_transform to convert GPS data to odometry and to convert latitude and longitude points to the robot's odometry coordinate system
	- GMapping to create a map and detect obstacles
	- move_base to navigate to the goals while avoiding obstacles (goals are set using recorded or inputted waypoints)

The outdoor_waypoint_nav package within waypoint_nav includes the following custom nodes:
	- gps_waypoint to read the waypoint file, convert waypoints to points in the map frame and then send the goals to move_base
	- gps_waypoint_continuous1 and gps_waypoint_continuous2 for continuous navigation between waypoints using two seperate controllers
	- collect_gps_waypoint to allow the user to drive the robot around and collect their own waypoints
	- calibrate_heading to set the heading of the robot at startup and fix issues with poor magnetometer data
	- plot_gps_waypoints to save raw data from the GPS for plotting purposes
	- gps_waypoint_mapping to combine waypoint navigation with Mandala Robotics' 3D mapping software for autonomous 3D mapping
  
  For additional information and instructions on how to use the package, see Clearpath Robotic's blog post and tutorial 
  
  	Tutorial: http://www.clearpathrobotics.com/assets/guides/husky/HuskyGPSWaypointNav.html
  	Blog: to be posted soon.
  
Video demonstrations can be found at my Youtube Channel: https://www.youtube.com/channel/UC3FoqSLn12-dKOQ1Sn0xbFQ/videos?view_as=subscriber

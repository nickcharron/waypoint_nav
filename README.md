# waypoint_nav

This package performs outdoor GPS waypoint navigation. It can navigate while building a map, avoiding obstacles, and can navigate continuously between each goal or stop at each goal. 

This repo is made to run on a Clearpath Husky with IMU, Novatel GPS, and Sick lms111 lidar.

This package uses a combination of the following packages:
* ekf_localization to fuse odometry data with IMU and GPS data
* navsat_transform to convert GPS data to odometry and to convert latitude and longitude points to the robot's odometry coordinate system
* GMapping to create a map and detect obstacles
* move_base to navigate to the goals while avoiding obstacles (goals are set using recorded or inputted waypoints)

The outdoor_waypoint_nav package within waypoint_nav includes the following custom nodes:
	
* gps_waypoint to read the waypoint file, convert waypoints to points in the map frame and then send the goals to move_base
* gps_waypoint_continuous1 and gps_waypoint_continuous2 for continuous navigation between waypoints using two seperate controllers
* collect_gps_waypoint to allow the user to drive the robot around and collect their own waypoints
* calibrate_heading to set the heading of the robot at startup and fix issues with poor magnetometer data
* plot_gps_waypoints to save raw data from the GPS for plotting purposes
* gps_waypoint_mapping to combine waypoint navigation with Mandala Robotics' 3D mapping software for autonomous 3D mapping
  
  
Video demonstrations can be found here:
* https://www.youtube.com/watch?v=2AUx9podA7c
* https://www.youtube.com/watch?v=k4ze0zAAPuQ
* https://www.youtube.com/watch?v=rAV1Uf6x1qQ
* https://www.youtube.com/watch?v=V5jG9hEXSV8
* https://www.youtube.com/watch?v=jsR8gYgeDG0

IMPORTANT NOTES:
----------------
 * Please DO NOT contact Clearpath Robotics with questions about this package. Instead, email me at nicholas.c.charron@gmail.com.
 
 * Regarding the calibration node: the heading calibration node is not required if your magnetometer is calibrated correctly. Instructions for performing magnetometer calibration are hard to come by, so often they are not calibrated correctly. Use this "hack" heading calibration node only if your heading is not correct when launching the waypoint navigation node (see this video: https://youtu.be/jsR8gYgeDG0). This will only temporarily solve your problem and must be run every time you start this package. We recommend instead to perform proper magnetometer calibration.
 
 * The continuous waypoint navigation software was tested in simulation and works well in simulation, however it still doesn't work properly with our tests outdoor.
 
 * Please submit pull requests if you update this package and/or fix bugs.
 
 * This code is meant as a tutorial to perform waypoint navigation using common ROS packages. It has not been tested for robustness and should not be used as a final product. If you are looking for a final working GPS waypoint navigation solution, please contact Clearpath Robotics, as they have recently developped a more commercial solution.

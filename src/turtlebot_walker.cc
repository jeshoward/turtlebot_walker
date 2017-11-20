/*
 * 
 * @author Jessica Howard
 * @file turtlebot_walker/src/turtlebot_walker.cc
 * @brief Algorithm to make the turtlebot move around a gazebo workspace
 * @detail The turtlebot will move forward in a straight line until
 * encountering an obstacle. The turtlebot will then rotate until a clear
 * path is available and again drive forward. Rinse and repeat until you
 * get bored of watching.
 *
 * 
 * @copyright Copyright (C) 2017, Jessica Howard
 * @license 3-Clause BSD License
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "math.h"
#include "ros/console.h"

//Laser scan message to detect obstacles
sensor_msgs::LaserScan scan_message_;

//Publisher for velocity messages
ros::Publisher velocity_publisher_;

//Subscriber to receive laser scan messages
ros::Subscriber scan_subscriber_;

//Subscriber to receive robot position
ros::Subscriber pose_subscriber_;

//Position and velocity messages
nav_msgs::Odometry odometry_pose_;
nav_msgs::Odometry obstacle_pose_;

//Callback for the laser scan
void scan_callback(sensor_msgs::LaserScan scan_message);

//Callback for the position and velocity message
void pose_callback(const nav_msgs::Odometry::ConstPtr & pose_message);

//Helper function to get distance between two points
double get_distance(double x1, double y1, double x2, double y2);

//Function to make the robot autonomously walk around the map
void walk();

//Function to avoid obstacles
void avoid_obstacle();

int main(int argc, char **argv) {
	//Initiate our turtlebot_walker
	ros::init(argc, argv, "turtlebot_walker");
	ros::NodeHandle node_handle;
	
	//Subscribe to the laser scanner
	scan_subscriber_ = node_handle.subscribe("/scan", 10, scan_callback);
	
	//Subscribe to the odometry to get the robot's position and velocity
	pose_subscriber_ = node_handle.subscribe("odom", 10, pose_callback);
	
	//Register the velocity publisher
	velocity_publisher_ = node_handle.advertise<geometry_msgs::Twist>
		("/cmd_vel_mux/input/teleop", 1000);
	
	ros::spinOnce();
	ros::Rate loop(1);
	loop.sleep();
	ros::spinOnce();
	walk();
	ros::spin();
	return 0;
}

/* 
 * @brief Callback for the laser scan
 */
void scan_callback(const sensor_msgs::LaserScan scan_message) {
	scan_message_ = scan_message;
}

/* 
 * @brief Callback for the position and velocity message 
 */
void pose_callback(const nav_msgs::Odometry::ConstPtr & pose_message) {
	//Save our robot's current position
	odometry_pose_.pose.pose.position.x = pose_message->pose.pose.position.x;
	odometry_pose_.pose.pose.position.y = pose_message->pose.pose.position.y;
	odometry_pose_.pose.pose.position.z = pose_message->pose.pose.position.z;
	
	//Save our robot's current orientation
	odometry_pose_.pose.pose.orientation.w = pose_message->pose.pose.orientation.w;
	odometry_pose_.pose.pose.orientation.x = pose_message->pose.pose.orientation.x;
	odometry_pose_.pose.pose.orientation.y = pose_message->pose.pose.orientation.y;
	odometry_pose_.pose.pose.orientation.z = pose_message->pose.pose.orientation.z;
}

/* 
 * @brief Helper function to get distance between two points 
 * @param x1 x coordinate of first point
 * @param y1 y coordinate of first point
 * @param x2 x coordinate of second point
 * @param y2 y coordinate of second point
 * @return returns the distance between (x1,y1) and (x2, y2) 
 */
double get_distance(double x1, double y1, double x2, double y2) {
	return sqrt(pow((x1-x2),2) + pow((y1-y2),2));
}

/* 
 * @brief Function to make the robot autonomously walk around the map 
 * @detail The turtlebot will move straight ahead until it encounters an obstacle,
 * when it gets close to the obstacle it will rotate until a clear path ahead is found
 * then it will continue to drive forward.
 */
void walk() {
	//generate a message to tell the robot where to go
	geometry_msgs::Twist velocity_message;
	
	//set our robot at a specific starting point
	
	
	ros::Rate loop_rate(100);
	
	bool obstacle = false;

	do {
		if(fmin(scan_message_.ranges) < 1.5) {
			velocity_message.linear.x = 0;
			velocity_message.angular.z = 0.35;
		} else {
			velocity_message.linear.x = 0.05;
			velocity_message.angular.z = 0;
		}

		velocity_publisher_.publish(velocity_message);
		ros::spinOnce();
		
	} while(ros::ok());
}


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

#ifndef TURTLEBOTWALKER_CLASS_H_
#define TURTLEBOTWALKER_CLASS_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "math.h"
#include "ros/console.h"

class TurtlebotWalker {
public:
	/*
	 * @brief constructor for the TurtlebotWalker
	 * @param Takes a node handle
	 */
	TurtlebotWalker(ros::NodeHandle* node_handle);

private:
	/*
	 * @brief node handle
	 */
	ros::NodeHandle node_handle_;
	
	/*
	 * @brief initialize subscribers to scan and odom
	 */
	void initialize_subscribers();
	
	/*
	 * @brief initialize publisher to cmd_vel_mux/input/teleop
	 */
	void initialize_publishers();
	
	/*
	 * @brief callback function for the laser scanner
	 */
	void scan_callback(const sensor_msgs::LaserScan::ConstPtr & scan_message);
	
	/*
	 * @brief Callback for the position and velocity message
	 */
	void pose_callback(const nav_msgs::Odometry::ConstPtr & pose_message);
	
	/*
	 * @brief Scan message to get distances to objects
	 */
	sensor_msgs::LaserScan scan_message_;
	
	/*
	 * @brief Twist message to change velocities
	 */
	geometry_msgs::Twist velocity_message_;

	/*
	 * @brief Publisher for velocity messages
	 */
	ros::Publisher velocity_publisher_;
	
	/*
	 * @brief Subscriber for scan messages
	 */
	ros::Subscriber scan_subscriber_;
	
	/*
	 * @brief Subscriber for odometry poses
	 */
	ros::Subscriber pose_subscriber_;
	/*
	 * @brief Position and velocity messages
	 */
	nav_msgs::Odometry odometry_pose_;	
};

#endif
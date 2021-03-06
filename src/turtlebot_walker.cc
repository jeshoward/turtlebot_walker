/*
 * @copyright Copyright (C) 2017, Jessica Howard
 * @author Jessica Howard
 * @file turtlebot_walker/src/turtlebot_walker.cc
 * @brief Algorithm to make the turtlebot move around a gazebo workspace
 * @detail The turtlebot will move forward in a straight line until
 * encountering an obstacle. The turtlebot will then rotate until a clear
 * path is available and again drive forward. Rinse and repeat until you
 * get bored of watching.
 *
 * 
 * 
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

#include "turtlebot_walker/turtlebot_walker.h"

TurtlebotWalker::TurtlebotWalker(ros::NodeHandle* node_handle) :
node_handle_(*node_handle) {
    ROS_INFO("Constructing TurtlebotWalker");

    // Initialize our publishers and subscribers
    initialize_subscribers();
    initialize_publishers();

    // Set default velocity values
    velocity_message_.linear.x = 0;
    velocity_message_.linear.y = 0;
    velocity_message_.linear.z = 0;
    velocity_message_.angular.x = 0;
    velocity_message_.angular.y = 0;
    velocity_message_.angular.z = 0;
}

void TurtlebotWalker::initialize_subscribers() {
    ROS_INFO("Initializing TurtlebotWalker subscribers.");
    // Subscribe to the laser scanner
    scan_subscriber_ =
        node_handle_.subscribe("scan", 10,
                               &TurtlebotWalker::scan_callback, this);

    // Subscribe to the odometry to get the robot's position and velocity
    pose_subscriber_ =
        node_handle_.subscribe("odom", 10,
                               &TurtlebotWalker::pose_callback, this);
}

void TurtlebotWalker::initialize_publishers() {
    ROS_INFO("Initializing TurtlebotWalker publishers.");
    // Register the velocity publisher
    velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>
        ("/cmd_vel_mux/input/teleop", 1000, true);
}

/* 
 * @brief Callback for the laser scan
 */
void TurtlebotWalker::scan_callback(const sensor_msgs::LaserScan::ConstPtr &
                                    scan_message) {
    scan_message_.ranges = scan_message->ranges;

    float min_range = scan_message_.ranges[0];
    float max_range = scan_message_.ranges[0];

    for (float range : scan_message_.ranges) {
        if (range < min_range && range > scan_message->range_min)
            min_range = range;
        if (range > max_range && range < scan_message->range_max)
            max_range = range;
    }

    if (min_range < 0.6) {
        // WE'RE GOING TO CRASH
        ROS_INFO("Crash imminent. ABORT ABORT!");
        velocity_message_.linear.x = -1;
        velocity_message_.angular.x = 1;
    } else if (min_range < 1) {
        // OBSTACLE AHEAD, SPIN!
        ROS_INFO("Encountered an obstacle. Spinning to avoid.");
        velocity_message_.linear.x = 0;
        velocity_message_.angular.z = 0.35;
    } else {
        ROS_DEBUG("Moving ahead.");
        // Looks clear
        velocity_message_.linear.x = 0.5;
        velocity_message_.angular.z = 0;
    }

    velocity_publisher_.publish(velocity_message_);
}

/* 
 * @brief Callback for the position and velocity message 
 */
void TurtlebotWalker::pose_callback(const nav_msgs::Odometry::ConstPtr &
                                    pose_message) {
    // Save our robot's current position
    odometry_pose_.pose.pose.position.x =
        pose_message->pose.pose.position.x;
    odometry_pose_.pose.pose.position.y =
        pose_message->pose.pose.position.y;
    odometry_pose_.pose.pose.position.z =
        pose_message->pose.pose.position.z;

    // Save our robot's current orientation
    odometry_pose_.pose.pose.orientation.w =
        pose_message->pose.pose.orientation.w;
    odometry_pose_.pose.pose.orientation.x =
        pose_message->pose.pose.orientation.x;
    odometry_pose_.pose.pose.orientation.y =
        pose_message->pose.pose.orientation.y;
    odometry_pose_.pose.pose.orientation.z =
        pose_message->pose.pose.orientation.z;
}

int main(int argc, char **argv) {
    ROS_INFO("Initiating TurtlebotWalker.");
    // Initiate our turtlebot_walker
    ros::init(argc, argv, "turtlebot_walker");
    ros::NodeHandle node_handle;

    // Create our TurtlebotWalker
    TurtlebotWalker turtle(&node_handle);

    ros::spin();

    return 0;
}

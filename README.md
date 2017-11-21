# ENPM808X - turtlebot_walker
Makes a turtlebot walk around a map autonomously while avoiding obstacles.

## Getting Started

### Installing Dependencies
The beginner tutorial has several dependencies including ROS Kinetic Kame and Catkin.

#### Installing ROS Kinetic Kame
Full instructions for installing ROS Kinetic Kame can be found [here](http://wiki.ros.org/kinetic/Installation).

The following instructions are for an Ubuntu installation.

1. Set up your sources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

2. Set up your keys
```http://wiki.ros.org/gtest
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

3. Installation
Make sure your Debian package is up to date:
```
sudo apt-get update
```

Perform a full desktop install:
```
sudo-apt-get install ros-kinetic-desktop-full
```

4. Initialize rosdep
```
sudo rosdep init
rosdep update
```

5. Environment set up
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

6. Install rosinstall
```
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

#### Installing Catkin
Full instructions for installing Catkin can be found [here](www.ros.org/wiki/catkin#Installing_catkin).

```
sudo apt-get install ros-lunar-catkin
```

#### Installing the turtlebot_gazebo dependency
To install the turtlebot_gazebo dependency:
```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```

#### Installing the turtlebot_walker from this repository
You will want to navigate to your Catkin workspace's src directory
```
cd ~/catkin_ws/src
```
This is the location you will want to add your new ROS package.
```
git clone -b master https://github.com/jeshoward/turtlebot_walker
```

Navigate back up to your Catkin workspace directory:
```
cd ~/catkin_ws/
```

Next, you will want to build the project:
```
catkin_make
```

### Running turtlebot_walker
The entire program can be executed from a terminal window using a single launch command:
```
roslaunch turtlebot_walker turtlebot_walker_launch.launch
```

### Recording to a rosbag
The launch file now supports bagfile recording. Recording is by default disabled, to enable it for 30 seconds use the following command during launch:
```
roslaunch turtlebot_walker turtlebot_walker_launch.launch record_bag:=true
```
The launch file records all topics except for the /camera/* topics.

### Playing a rosbag
To play the bag you've just recorded, enter the following:
```
rosbag play <rosbag_file_name>
```
Please note that you cannot play the rosbag file while Gazebo is still running.

To merely inspect the rosbag to find out information about what topics and types it recorded:
```
rosbag info <rosbag_file_name>
```

## License
BSD 3-Clause License

Copyright (c) 2017, Jessica Howard
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

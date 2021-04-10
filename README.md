# ROS
ROS Projects

Installation Steps
1. Setup your computer to accept software from packages.ros.org.

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

2. Set up your keys.

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

3. Make sure your Debian package index is up-to-date.

sudo apt update

4. Installing the ROS recommended configuration.

sudo apt install ros-melodic-desktop-full


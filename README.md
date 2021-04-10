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

Config Steps

1. Adding environment variables: To Automatically add ROS environment variables to your bash session every time a new shell (terminal) is launched, enter the following commands (this step is similar as adding environmental variable in windows):

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

2. Initialize rosdep: Before you can use many ROS tools, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS.

sudo apt install python-rosdep
sudo rosdep init
rosdep update

More packages to install

#Catkin Tools

sudo apt-get install ros-melodic-catkin python-catkin-tools

#std_msg package

sudo apt install ros-melodic-std-msgs

#turtlesim

sudo apt-get install ros-melodic-ros-tutorials

#Gmapping package: ​

sudo apt-get install ros-melodic-gmapping

#Teleop keyboard package: (Ignore if already installed)

sudo apt-get install ros-melodic-teleop-twist-keyboard

#Navigation package: ​

sudo apt-get install ros-melodic-navigation

#Tf2 package:

sudo apt-get install ros-melodic-tf2-sensor-msgs

#AMCL package: ​

sudo apt-get install ros-melodic-amcl

#Map server package:

sudo apt-get install ros-melodic-map-server



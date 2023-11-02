# Requirement for this package
1. Ubunut 20.04
2. Install Ardupilot SITL
3. Install Gazebo
4. Install ROS Neotic

# Steps to setup the package

```
mkdir -p ardupilot_ws/src
cd ardupilot_ws
catkin init
```
The Workscape should me valid
Exctract the package in src folder

```
cd ~/ardupilot_ws
catkin_make
source ~/ardupilot_ws/devel/setup.bash
```
Use this command to prevent soucring setup.bash each time a new terminal is opened
```
echo 'source ~/ardupilot_ws/devel/setup.bash' >> ~/.bashrc
```
### Terminal 1
```
cd ardupilot/arduCopter
gz sim -v4 -r iris_runway.sdf
```

### Terminal 2
```
cd ardupilot/arduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

### Terminal 3
```
cd ardupilot_ws
roslaunch task apm.launch
```
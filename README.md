# offboard

Offboard control using [ROS](http://www.ros.org) and [MAVROS](https://github.com/mavlink/mavros) for [PX4](https://github.com/PX4/Firmware).


## Usage

Can be used for offboard control. It also implements a path following control that can be used independently
form the project. 

### Dependencies

- [ROS](http://www.ros.org)
- [MAVROS](https://github.com/mavlink/mavros)
- [Catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

### Building

```
cd ~/some_path
git clone https://github.com/Stifael/offboard
cd ~/catkin_ws
ln -s ~/some_path/offboard ~/catkin_ws/src/offboard
catkin_make
```

### Running project

Start PX4 with e.g.:
```
make posix gazebo
```

Then start MAVROS:

```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

Start offboard 

```
roslaunch offboard offb.launch
```





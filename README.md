# smely-zajko-ros

## Installation
First clone this repository
```{r, engine='bash', count_lines}
$ git clone https://github.com/Robotics-DAI-FMFI-UK/smely-zajko-ros
```
### OpenCV

```{r, engine='bash', count_lines}
$ sudo apt-get install libopencv-dev
```

### putty-tools (plink)
```{r, engine='bash', count_lines}
$ sudo apt-get install putty-tools
```
Do not forget to add user to group `dialout`.
```{r, engine='bash', count_lines}
$ sudo usermod -a -G dialout <userName>
```
### ROS Kinetic
```{r, engine='bash', count_lines}
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full
$ sudo rosdep init
$ rosdep update
```
For more information follow ROS Kinetic [installation manual](http://wiki.ros.org/kinetic/Installation/Ubuntu).

### Setting up CLion

```{r, engine='bash', count_lines}
$ source devel/setup.bash && sh clion.sh &
```
Make sure your `PROJECT_DIR` is in `src` folder (without it, you will not be able to autocomplete)! You can check it in `.idea/misc.xml`.

## Building
From root folder run `$ catkin_make`. Make sure you run `source devel/setup.bash` before.

## Running
From root folder on PC run `$ ./startme`. This launches the ros master

From root folder on Jetson run `$ ./jetson_camera_start`. Camera node will start and connect to the master


## Record run via rosbag
We can record whole run and play it again (with correct timing, data, etc.) via `rosbag`. To record all nodes (not recommended) in run use
```{r, engine='bash', count_lines}
$ rosbag record -a
```
but preferable way is to record only sensors with
```{r, engine='bash', count_lines}
$ rosbag record /sensors_publishers/camera/image /sensors_publishers/gps_publisher /sensors_publishers/houyo_publisher /sensors_publishers/gps_publisher /sensors_publishers/imu_publisher /sensors_publishers/sbot_publisher
```
and for play 
```{r, engine='bash', count_lines}
$ rosbag play filename.bag
```
`*.bag` files can be quite big, so use `rosbag` to compress and decompress files
```{r, engine='bash', count_lines}
$ rosbag compress *.bag
$ rosbag decompress *.bag
```
For more information check [documentation](http://wiki.ros.org/rosbag/Commandline).

# smely-zajko-ros

## Installation

### OpenCV

```{r, engine='bash', count_lines}
$ sudo apt-get install libopencv-dev
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

# Robust Visual-Aided Autonomous Takeoff, Tracking and Landing of a small UAV on a Moving Landing Platform for Life-Long Operation

Authors: [Pablo R. Palafox](https://pablorpalafox.github.io/), Mario Garzón, João Valente, Juan Jesús Roldán and Antonio Barrientos


## 1. Prerequisites

We have tested our system on ROS Kinetic and Ubuntu 16.04.

You can follow [this](http://wiki.ros.org/kinetic/Installation/Ubuntu) tutorial to install ROS Kinetic.

Once you're done, you can install all required ROS packages for this project by running the following command:

```bash
$ chmod +x install_dependencies.sh
$ ./install_dependencies.sh
```

## 2. Usage

Create a catkin workspace and move into it. Then clone this repository into a folder named `src`. Finally, go back to `ws`, build the workspace and source your environment:

```bash
$ mkdir ws && cd ws
$ git clone git@github.com:pablorpalafox/uav-autonomous-landing.git src
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

The following instructions correspond to the simulated environment. (Documentation for the real environment will be released soon.)

Launch the world with both the UGV and the UAV:

```bash
$ roslaunch takeoff both.launch
```

In a different terminal (don't forget to source the environment everytime you open a new terminal by using `cd ws && source devel/setup.bash`), launch the detection and tracking modules:

```bash
$ roslaunch uav_vision detection_tracking.launch
```

Now you can start moving the Summit (UGV) by launching the corresponding node `summit_moves`:

```bash
$ roslaunch summit_moves summit_moves.launch
```

### 

## 3. License

Libraries within `thirdparty` are released with their own license.

The rest of the code in this repository is released under an [MIT License](LICENSE). 

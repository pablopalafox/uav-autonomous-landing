# Robust Visual-Aided Autonomous Takeoff, Tracking and Landing of a small UAV on a Moving Landing Platform for Life-Long Operation

Created by [Pablo R. Palafox](https://pablorpalafox.github.io/), Mario Garzón, João Valente, Juan Jesús Roldán and Antonio Barrientos

<p align="center">
  <img src="/docs/landing.gif" alt="">
</p>

### Publication
This code is the implementation of the following [paper](https://www.mdpi.com/2076-3417/9/13/2661)

If you find our work useful in your research, please cite:

         @article{palafox2019robust,
           title={Robust Visual-Aided Autonomous Takeoff, Tracking, and Landing of a Small UAV on a Moving Landing Platform for Life-Long Operation},
           author={Palafox, Pablo R and Garz{\'o}n, Mario and Valente, Jo{\~a}o and Rold{\'a}n, Juan Jes{\'u}s and Barrientos, Antonio},
           journal={Applied Sciences},
           volume={9},
           number={13},
           pages={2661},
           year={2019},
           publisher={Multidisciplinary Digital Publishing Institute}
         }
   
### Abstract
Robot cooperation is key in Search and Rescue (SaR) tasks. Frequently, these tasks take place in complex scenarios affected by different types of disasters, so an aerial viewpoint is useful for autonomous navigation or human tele-operation. In such cases, an Unmanned Aerial Vehicle (UAV) in cooperation with an Unmanned Ground Vehicle (UGV) can provide valuable insight into the area. To carry out its work successfully, such as multi-robot system requires the autonomous takeoff, tracking, and landing of the UAV on the moving UGV. Furthermore, it needs to be robust and capable of life-long operation. In this paper, we present an autonomous system that enables a UAV to take off autonomously from a moving landing platform, locate it using visual cues, follow it, and robustly land on it. The system relies on a finite state machine, which together with a novel re-localization module allows the system to operate robustly for extended periods of time and to recover from potential failed landing maneuvers. Two approaches for tracking and landing are developed, implemented, and tested. The first variant is based on a novel height-adaptive PID controller that uses the current position of the landing platform as the target. The second one combines this height-adaptive PID controller with a Kalman filter in order to predict the future positions of the platform and provide them as input to the PID controller. This facilitates tracking and, mainly, landing. Both the system as a whole and the re-localization module in particular have been tested extensively in a simulated environment (Gazebo). We also present a qualitative evaluation of the system on the real robotic platforms, demonstrating that our system can also be deployed on real robotic platforms.


Click on the image below to watch a [VIDEO](https://youtu.be/CCrPBw_we2E) demonstrating the system:

[![https://youtu.be/CCrPBw_we2E](http://img.youtube.com/vi/CCrPBw_we2E/0.jpg)](https://youtu.be/CCrPBw_we2E)


## 1. Prerequisites

We have tested our system on ROS Kinetic and Ubuntu 16.04.

You can follow [this](http://wiki.ros.org/kinetic/Installation/Ubuntu) tutorial to install ROS Kinetic.


## 2. Usage

Create a catkin workspace and move into it. Then clone this repository into a folder (e.g., `src`). We'll install some ROS dependencies and, finally, go back to `ws`, build the workspace and source your environment. The individual steps are:

```bash
$ mkdir ws && cd ws
$ git clone git@github.com:pablorpalafox/uav-autonomous-landing.git src
```

Install all required ROS packages for this project by running:

```bash
$ cd src
$ chmod +x install_dependencies.sh
$ ./install_dependencies.sh
```

Finally, compile the project and source your environment:

```bash
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

The following instructions correspond to the simulated environment. (Documentation for the real environment will be released soon.)

Connect a PS3 joystick to your computer and verify that the port it has been assigned to matches that defined in [uav-autonomous-landing/takeoff/launch/ardrone_teleop.launch](takeoff/launch/ardrone_teleop.launch), i.e., `value="/dev/input/js0"`. You can install `jstest-gtk` to test your joystick / controller:

```bash
$ sudo apt-get update
$ sudo apt-get install jstest-gtk
```

Now launch the world with both the UGV and the UAV:

```bash
$ roslaunch takeoff both.launch
```

In a different terminal (don't forget to source the environment everytime you open a new terminal by using `cd ws && source devel/setup.bash`), launch the detection and tracking modules:

```bash
$ roslaunch uav_vision detection_tracking.launch
```

Finally, also in a different terminal (don't forget `source devel/setup.bash`), launch the Kalman prediction module:

```bash
$ roslaunch ped_traj_pred kalman_pred.launch
```

The node `ped_traj_pred` will be listening to the landing platform's centroid topic (`/platform/current_platform_position_in_world`), published by the `platform_detection` node. It will predict a vector of future positions of the landing platform (the first item in this vector is the current position, so that we can also use a non-predictive system). You can decide whether to use a predictive or a non-predictive system by toggling the `use_pred` parameter in the trackbar that pops up when launching `detection_tracking.launch`.

Now you can start moving the Summit (UGV) and the UAV by pressing `L1` on your PS3 joystick. By default, the drone should start taking off, then land automatically after some seconds and take off again after a couple of seconds on the platform. This is the default mission, but feel free to design your own!

### 

## 3. License

Libraries within `thirdparty` are released with their own license.

The rest of the code in this repository is released under an [MIT License](LICENSE). 

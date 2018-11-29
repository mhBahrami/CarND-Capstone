This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Team Members

* [Xiaoqi Li](https://github.com/Charlie-Xiaoqi) - team lead
* [Mohammad Hossein Bahramimanesh](https://github.com/mhBahrami) - trajectory planner, controller and integrator
* [Nikola Noxon](https://github.com/nikolanoxon) - traffic light detection
* [Tamoghna Das](https://github.com/tamoghna21) - traffic light detection and controller

### Project Components

* diagram of nodes and messages

#### Visualization

[![visualization-video](https://github.com/erlink.png)](https://www.youtube.com)

#### Traffic Light Detection

* We adopted the inception v2 model from the tensorflow zoo, the architecture of the model can be found over here: 
[Inception v2 architecture](https://towardsdatascience.com/a-simple-guide-to-the-versions-of-the-inception-network-7fc52b863202)

#### Traffic Light Classification

| Simulator model         | Test site model                            |
|:-----------------------:|:--------------------------------------:|
| ![simulator model](https://github.com/Charlie-Xiaoqi/CarND-Capstone-1/blob/master/Classification_image/Simulator_image.png)          | ![Test site model](https://github.com/Charlie-Xiaoqi/CarND-Capstone-1/blob/master/Classification_image/Testsite_image.png)

* Four output classes: GREEN, YELLOW, RED, NONE.
* Test accuracy: lowest precision was red at 98%, lowest recall was green at 88%.
* See model and training code in [Nikola's repo](https://github.com/nikolanoxon/CarND-Traffic-Light-Classifier) in iPython notebooks.
* See inference code in [tl\_classifier.py](https://github.com/Charlie-Xiaoqi/CarND-Capstone-1/blob/master/ros/src/tl_detector/light_classification/tl_classifier.py).

#### Trajectory Planner

* Deceleration profiles are governed a set of kinematic equations of motion in function get_stop_distance(self) inside the code [waypoint\_updater.py](https://github.com/Charlie-Xiaoqi/CarND-Capstone-1/blob/master/ros/src/waypoint_updater/waypoint_updater.py).
* Safe distance is calculated based on car current speed.
* If further than safe distance from the traffic light, the car ignores the traffic light's color and accelerates up to cruising speed.
* Within safe distance of a red traffic light, car decelerates at whatever rate would result in stopping exactly on the stop line. Otherwise the car will keep going.
* When car within stopping distance (16 meters) of yellow, it start to decelerate.

#### Control Subsystem

This subsystem publishes control commands for the vehicle’s steering, throttle, and brakes based on a list of waypoints to follow.

#### Waypoint Follower Node

This node was given to us by Udacity. It parses the list of waypoints to follow and publishes proposed linear and angular velocities to the /twist_cmd topic

#### Drive By Wire (DBW) Node

The DBW node is the final step in the self driving vehicle’s system. At this point we have a target linear and angular velocity and must adjust the vehicle’s controls accordingly. In this project we control 3 things: throttle, steering, brakes. As such, we have 3 distinct controllers to interface with the vehicle.

#### Throttle Controller

The throttle controller is a simple PID controller that compares the current velocity with the target velocity and adjusts the throttle accordingly. The throttle gains were tuned using trial and error for allowing reasonable acceleration without oscillation around the set-point.

#### Steering Controller

This controller translates the proposed linear and angular velocities into a steering angle based on the vehicle’s steering ratio and wheelbase length. To ensure our vehicle drives smoothly, we cap the maximum linear and angular acceleration rates. The steering angle computed by the controller is also passed through a low pass filter to reduce possible jitter from noise in velocity data.

#### Braking Controller

This is the simplest controller of the three - we simply proportionally brake based on the throttle and the brake deadband.

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

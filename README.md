## Description ##
roboy-ros-control provides ros control hierarchy for roboy (v2.0) hardware.

# Building from source #
The following instructions guide you through the process of building this repo from source.
## Dependencies
### git 
```
#!bash
sudo apt install git
```
### doxygen[OPTIONAL]
```
#!bash
sudo apt install doxygen
```
### [ROS kinetic](http://wiki.ros.org/kinetic/)
For detailed description of installation see [here](http://wiki.ros.org/kinetic/Installation/Ubuntu). 
```
#!bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
```
#### install ros desktop and control related stuff
```
#!bash
sudo apt install ros-kinetic-desktop
sudo apt install ros-kinetic-controller-interface ros-kinetic-controller-manager ros-kinetic-control-toolbox ros-kinetic-transmission-interface ros-kinetic-joint-limits-interface ros-kinetic-ecl-geometry ros-kinetic-gazebo-ros-control
```
#### install packages from roboy.org
```
#!bash
sudo apt install libcmaes
```
#### install gazebo7 and gazebo-ros-pkgs
```
#!bash
sudo apt-get install gazebo7 libgazebo7-dev
sudo apt-get install ros-kinetic-gazebo-ros-pkgs
```
You should try to run gazebo now, to make sure its working.
```
#!bash
source /usr/share/gazebo-7.0/setup.sh
gazebo --verbose
```
If you seen an output like, 'waiting for namespace'...'giving up'. Gazebo hasn't been able to download the models. You will need to do this manually. Go to the osrf [bitbucket](https://bitbucket.org/osrf/gazebo_models/downloads), click download repository. Then unzip and move to gazebo models path:
```
#!bash
cd /path/to/osrf-gazebo_models-*.zip
unzip osrf-gazebo_models-*.zip -d gazebo_models
mv gazebo_models/osrf-gazebo_models-*/* ~/.gazebo/models
```
If you run gazebo now it should pop up without complaints and show an empty world.

## clone repos
```
#!bash
git clone --recursive https://github.com/Roboy/roboy-ros-control
cd roboy-ros-control
```
## Build
### Environmental variables and sourceing
Now this is very important. For both build and especially running the code successfully you will need to define some env variables and source some stuff. Add the following lines to your ~/.bashrc (adjusting the paths to your system):
```
#!bash
source /usr/share/gazebo-7.0/setup.sh
export GAZEBO_MODEL_PATH=/path/to/roboy-ros-control/src/roboy_models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=/path/to/roboy-ros-control/devel/lib:$GAZEBO_PLUGIN_PATH
export GAZEBO_RESOURCE_PATH=/path/to/roboy-ros-control/src/roboy_models:$GAZEBO_RESOURCE_PATH
source /opt/ros/kinetic/setup.bash
source /path/to/roboy-ros-control/devel/setup.bash
```
Then you can build with:
```
#!bash
source ~/.bashrc
cd path/to/roboy-ros-control
catkin_make --pkg common_utilities
source devel/setup.bash
catkin_make
```
### symlink to meshes
For gazebo to find the meshes, create a symlink:
```
#!bash
mkdir -p ~/.gazebo/models
ln -s path/to/roboy-ros-control/src/roboy_models/legs_with_upper_body ~/.gazebo/models/
ln -s path/to/roboy-ros-control/src/roboy_models/PaBiRoboy ~/.gazebo/models/
ln -s path/to/roboy-ros-control/src/roboy_models/Roboy ~/.gazebo/models/
```
#### If the build fails, complaining about missing headers,
this is probably because ros cannot find the headers it just created. You need to source the devel/setup.bash:
```
#!bash
source devel/setup.bash
catkin_make
```
## Documentation ##
Generate a doxygen documentation using the following command:
```
#!bash
cd path/to/roboy-ros-control
doxygen Doxyfile
```
The documentation is put into the doc folder.

# docker
This repo is build into a docker image. Please follow the [docker install instructions](https://docs.docker.com/engine/installation/) for your system.
## usage
You can run eg the simulation with the following commands:
```
#!bash
xhost +local:root
docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" letrend/roboy-ros-control:devel roslaunch myo_master roboySim.launch
```
The xhost command enables GUI rendering, please check this [page](http://wiki.ros.org/docker/Tutorials/GUI) for alternatives.
The docker run command downloads the image and runs the following commands. Once you are done, disable xhost with:
```
#!bash
xhost -local:root
```
## build
You can also build your own docker image, using the Dockerfile in the repo with the following command:
```
#!bash
cd path/to/roboy-ros-control
docker build -t roboy-ros-control .
```
# Run it
## with real roboy
### start the controller_manager
```
#!bash
sudo -s                                                 # so far the openPowerLink stack can only be run as root
source /opt/ros/kinetic/setup.bash                      # You will probably need to to this, since the .bashrc is for each   source /path/to/roboy-ros/control/devel/setup.bash      # user, ie. not for root 
roslaunch roboy_managing_node roboy.launch
```
### initialise the controllers
The format here is based on the msg defined here: https://github.com/Roboy/common_utilities/blob/master/msg/Initialize.msg
You can initialise a number of controllers at the same time.
The values:
- id
- controlmode (defined in [common_utilities/include/CommonDefinitions.h](https://github.com/Roboy/common_utilities/blob/master/include/CommonDefinitions.h#L8))
- resource (aka joint_name in [roboy_controller.yaml](https://github.com/Roboy/roboy_controller/blob/master/config/roboy_controller.yaml))
 need to match the values defined in [roboy_controller.yaml](https://github.com/Roboy/roboy_controller/blob/master/config/roboy_controller.yaml)

**TIP:** Autocomplete is your friend if you use '/' i.e. `rostopic pub / -tab-`
```
#!bash
rostopic pub /roboy/initialize /common_utilities/Initialize "controllers: - {id: 0, controlmode: 0, resource: '', ganglion: 0, motor: 0}"
```

### compile a trajectory for the motors
For **every** motor you need to fill [/common_utilities/Trajectory](https://github.com/Roboy/common_utilities/blob/master/msg/Trajectory.msg) msgs, where samplerate is the frequency defining the time delta between waypoints.
```
#!bash
uint32 id
float32 samplerate
float32[] waypoints
```
The message then should be published on the topic: /roboy/trajectory/**your_joint_name** as defined in i.e. [ForceController.cpp](https://github.com/Roboy/roboy_controller/blob/master/src/ForceController.cpp#L32) (similar for the other controllers)

This will automatically stop the controller of the associated controller.
Do this for all motors you want to set trajectories for.

If you forgot your motor name you should be able to list all of the available ROS topics with:
```
rostopic list
```

### start the controllers
Now you can start **all controllers at once**:

A message on topic: `/roboy/steer_record` of type [/common_utilities/steer](https://github.com/Roboy/common_utilities/blob/master/msg/Steer.msg) with the options of this [enum](https://github.com/Roboy/common_utilities/blob/master/include/CommonDefinitions.h#L37):
- 0 (STOP_TRAJECTORY)
- 1 (PLAY_TRAJECTORY)
- 2 (PAUSE_TRAJECTORY)
- 3 (REWIND_TRAJECTORY)

Starts/stops/etc all controllers as defined here: https://github.com/Roboy/roboy_hardware/blob/master/src/roboy.cpp#L275

----

## with simulated roboy
```
#!bash
rviz &
rosrun roboy_simulation roboySim
```
In rviz you can add the walking plugin panel for controlling the simulation.

## Usage
List the availale controller types:
```
#!bash
rosservice call /controller_manager/list_controller_types
```
This should show our custom controller plugin:
```
#!bash
types: ['roboy_controller/Position_controller']
base_classes: ['controller_interface::ControllerBase']
```

### Status of the controller ###
The status of the controller can be queried via:
```
#!bash
rosservice call /controller_manager/list_controllers
```
This should show the stopped controllers, together with the resources they have been assigned to
```
#!bash
controller:
  -
    name: motor0
    state: stopped
    type: roboy_controller/PositionController
    hardware_interface: hardware_interface::PositionJointInterface
    resources: ['motor0']
  -
    name: motor1
    state: stopped
    type: roboy_controller/VelocityController
    hardware_interface: hardware_interface::VelocityJointInterface
    resources: ['motor1']
  -
    name: motor3
    state: stopped
    type: roboy_controller/ForceController
    hardware_interface: hardware_interface::EffortJointInterface
    resources: ['motor3']

```

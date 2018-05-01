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

#### other dependencies
```
#!bash 
sudo apt install ros-kinetic-controller-manager ros-kinetic-gazebo-ros-control
sudo apt install libpcap-dev libjson-glib-dev ros-kinetic-rosjava python-pyaudio maven
```
## clone repo with submodules
```
#!bash
git clone --recursive https://github.com/Roboy/Roboy
cd Roboy
```
## Build
### Environmental variables and sourceing
Now this is very important. For both build and especially running the code successfully you will need to define some env variables and source some stuff. Add the following lines to your ~/.bashrc (adjusting the paths to your system):
```
#!bash
source /usr/share/gazebo-7.0/setup.sh
export GAZEBO_MODEL_PATH=/path/to/Roboy/src/roboy_models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=/path/to/Roboy/devel/lib:$GAZEBO_PLUGIN_PATH
export GAZEBO_RESOURCE_PATH=/path/to/Roboy/src/roboy_models:$GAZEBO_RESOURCE_PATH
source /opt/ros/kinetic/setup.bash
```
Then you can build with:
```
#!bash
source ~/.bashrc
cd path/to/Roboy
catkin_make
```
### symlink to meshes
For gazebo to find the meshes, create a symlink:
```
#!bash
mkdir -p ~/.gazebo/models
ln -s path/to/Roboy/src/roboy_models/legs_with_upper_body ~/.gazebo/models/
ln -s path/to/Roboy/src/roboy_models/PaBiRoboy ~/.gazebo/models/
ln -s path/to/Roboy/src/roboy_models/Roboy ~/.gazebo/models/
```
#### If the build fails, complaining about missing headers,
this is probably because ros cannot find the headers it just created. You need to source the devel/setup.bash:
```
#!bash
source devel/setup.bash
catkin_make
```


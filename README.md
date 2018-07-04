# Building from source #
The following instructions guide you through the process of building this repo from source. 

## Ubuntu, CUDA and ZED

In order work on the Roboy_vision project, CUDA version 9.1. must be installed. Optionally, you can also remove this subproject in case it is not needed. 

### Linux version

CUDA only works on Ubuntu 16.04 kernel version 04.04, but unofficially kernel versions up to 04.10 support it. Newer kernels (04.13) will definitely cause errors. Download all kernel files from [this website] (http://kernel.ubuntu.com/~kernel-ppa/mainline/): 


    linux-headers-xxxxxx-generic-xxxxxx_amd64.deb

    linux-headers-xxxxxx_all.deb

    linux-image-xxxxxx-generic-xxxxxx_amd64.deb

where the xxxxxx is replaced by the kernel version. Save all files in one folder, cd into it and execute following command: 

    sudo dpkg -i *.deb

Restart ubuntu and choose the new kernel version from the GRUB menu (Press AND hold the Shift key when booting to make it appear in case that does not happen yet.) 

### CUDA 9.1
__Installation:__
Download CUDA files from the [nvidia website] (https://developer.nvidia.com/cuda-91-download-archive) - we also downloaded the updates. Choose the following target platform: Linux - x86_64 - Ubuntu - 16.04 - deb(local) and download all files to a folder cd into this folder. 

	sudo dpkg -i cuda-repo-ubuntu1604-9-1-local_9.1.85-1_amd64.deb
	# OR 
	sudo dpkg -i *.deb

	sudo apt-key add /var/cuda-repo-9-1-local/7fa2af80.pub
	sudo apt-get update
	sudo apt-get install cuda=9.1.85-1
   	sudo apt upgrade cuda

__Check:__
Make sure you have the right Cuda version installed by restarting your computer, checking if it still runs with a normal resolution (MIND TO CHOOSE THE RIGHT KERNEL!) and run: 

	apt policy cuda
	uname -r

Check if the numbers are correct. 

__In case of issues:__
In case you can't log-in due to graphics issues, uninstall Cuda. Do not try to blacklist any components, it should be working without this being necessary. Boot Ubuntu, on the log-in screen continue in a virtual console by pressing Ctrl + Alt + F1: 

	sudo service lightdm stop
	sudo apt-get remove --purge nvidia-387 nvidia-modprobe nvidia-settings 

### ZED 

In order work on the Roboy\_vision project, the ZED sdk must be installed. Optionally, you can also remove this subproject in case it is not needed. Choose the ZED sdk from the [stereolabs website] (https://www.stereolabs.com/developers/release/2.4/#sdkdownloads_anchor), choose the ZED SDK Linux Version for Cuda 9.1., download and install using: 

     chmod +x ZED_SDK_Linux_Ubuntu16_v2.4.1.run 
     ./ZED_SDK_Linux_Ubuntu16_v2.4.1.run 

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
sudo add-apt-repository ppa:letrend/libcmaes
sudo apt-get update
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
source /usr/share/gazebo-7/setup.sh
gazebo --verbose
```
If you seen an output like, 'waiting for namespace'...'giving up' or 'Error no Namespace found', Gazebo hasn't been able to download the models. You will need to do this manually. Go to the osrf [bitbucket](https://bitbucket.org/osrf/gazebo_models/downloads), click download repository. Then unzip and move to gazebo models path:
```
#!bash
cd /path/to/osrf-gazebo_models-*.zip
unzip osrf-gazebo_models-*.zip -d gazebo_models
mkdir -p ~/.gazebo/models
mv gazebo_models/osrf-gazebo_models-*/* ~/.gazebo/models
```
If you run gazebo now it should pop up without complaints and show an empty world as well as model insertion options on the left.

#### other dependencies
```
#!bash 
sudo apt install ros-kinetic-controller-manager ros-kinetic-gazebo-ros-control
sudo apt install libpcap-dev libjson-glib-dev ros-kinetic-rosjava python-pyaudio maven
sudo apt install ros-kinetic-robot-localization ros-kinetic-pcl-ros libalglib-dev 
sudo apt install qt5-default libopenni2-dev
```
## clone repo with submodules
```
#!bash
git clone --recursive https://github.com/Roboy/Roboy
cd Roboy
```
## Build
### Environmental variables and sourceing
Now this is __very important__. For both build and especially running the code successfully you will need to define some env variables and source some stuff. Add the following lines to your ~/.bashrc (adjust the paths to your system):
```
#!bash
source /usr/share/gazebo-7.0/setup.sh
export GAZEBO_MODEL_PATH=/path/to/Roboy/src/roboy_models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=/path/to/Roboy/devel/lib:$GAZEBO_PLUGIN_PATH
export GAZEBO_RESOURCE_PATH=/path/to/Roboy/src/roboy_models:$GAZEBO_RESOURCE_PATH

# 2 source options: 
source /opt/ros/kinetic/setup.bash
# OR for permanent source: 
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Then you can build with:
```
#!bash
source ~/.bashrc
cd path/to/Roboy
catkin_make
```

__In case of issues:__
In case catkin\_make does not exist, __DO NOT__ try to install the catkin package, catkin\_make is provided by ros\_kinetic_catkin as part of the ros kinetic installation. Follow [these instructions] (http://wiki.ros.org/kinetic/Installation/Source) instead and MAKE SURE YOU SOURCED IT!  
If the build fails due to missing components, try to find and install these. Most commonly, they can be found as a package of ros-kinetic-xxxxxx, where you replace the x-es by the name of the component and replace underscores with a hyphen. 

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

### The project should now build without any errors, nevertheless, some components won't work yet due to additional dependencies. Please refer to the respective submodel description / Readme and installation guidelines in this case. 

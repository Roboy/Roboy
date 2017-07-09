cmake_minimum_required(VERSION 2.8.3)
project(roboy_cognition_manager)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

## Find catkin macros and libraries
# find_package(catkin REQUIRED COMPONENTS
#   roboy_communication_control
#   roboy_speech_recognition
#   roboy_communication_cognition
#   speech_synthesis
# )

#  services in the 'srv' folder
# add_service_files(
#   FILES
#   ShowEmotion.srv
# )

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   EmotionMsg.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
  
)


#############
## Install ##
#############
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 # install(PROGRAMS
 #   scripts/unity_face.py
 #   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 # )

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


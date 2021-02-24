cmake_minimum_required(VERSION 2.8.3)
project(hast)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  apriltags_ros
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(OpenCV REQUIRED)
find_package(robot_plugins REQUIRED)
find_package(apriltags_ros REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
	ckfugv_sensor.msg
  stereougv_sensor.msg
  flag.msg
  matrix3x3.msg
  pixels.msg
  posewithcov.msg
  posewithheader.msg
  slam_poses.msg
	synctime.msg
  uavstate.msg
  uavmux.msg
  ugvmux.msg
	ugvmuxstatus.msg
	ugvposewithcov.msg
  ugvstate.msg
  vector3.msg
  vector4.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
	ckfshare.srv
  flipflop.srv
  servtime.srv
	servmuxmap.srv
	slamswitch.srv
  null.srv
  stereoodomcs.srv
  uavcontrolstate.srv
  uavpicket.srv
  uavnavstate.srv
  ugvautopilot.srv
  ugvdrive.srv
  ugvgoal.srv
	ugvugvsensor.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  apriltags_ros
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  classes/include
  ${catkin_INCLUDE_DIRS}
  ${apriltags_ros_INCLUDE_DIRS}
  ${robot_plugins_INCLUDE_DIRS}
  )

add_library(        utils  classes/src/utils.cpp) # Generic support functions
  add_dependencies( utils  hast_gencpp)
add_library(        fileops  classes/src/fileops.cpp) # File writing functions for creating matlab structures
  add_dependencies( fileops  hast_gencpp)

add_library(        hastAPRIL classes/src/apriltagclass.cpp)
  add_dependencies( hastAPRIL hast_gencpp)
add_library(        hastCKF  classes/src/ckfClass.cpp)
  add_dependencies( hastCKF  hast_gencpp)

add_library(        hastDKF  classes/src/DiscreteKF.cpp)
  add_dependencies( hastDKF  hast_gencpp)

add_library(            hastUAV  classes/src/hastUAV.cpp)
  target_link_libraries(hastUAV  utils fileops hastCKF)
  add_dependencies(     hastUAV  hast_gencpp)

add_library(            hastUGV  classes/src/hastUGV.cpp)
  target_link_libraries(hastUGV  utils fileops hastCKF hastDKF)
  add_dependencies(     hastUGV  hast_gencpp)

add_executable(         splitSLAM  src/splitSLAM.cpp)
  add_dependencies(     splitSLAM  hast_gencpp)
  target_link_libraries(splitSLAM  utils fileops hastCKF hastDKF  hastUAV hastUGV hastAPRIL ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(         ugvMuxxer src/ugvMuxxer.cpp)
  add_dependencies(     ugvMuxxer hast_gencpp)
  target_link_libraries(ugvMuxxer utils fileops hastCKF hastDKF  hastUAV hastUGV hastAPRIL ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(         ugvCKF    src/ugvCKF.cpp)
  add_dependencies(     ugvCKF    hast_gencpp)
  target_link_libraries(ugvCKF    utils fileops hastCKF hastDKF  hastUAV hastUGV hastAPRIL ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(         uavCKF    src/uavCKF.cpp)
  add_dependencies(     uavCKF    hast_gencpp)
  target_link_libraries(uavCKF    utils fileops hastCKF hastDKF  hastUAV hastUGV hastAPRIL ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(         trial  src/trial.cpp)
  add_dependencies(     trial  hast_gencpp)
  target_link_libraries(trial  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(         wait_for_trial_finish  src/wait_for_trial_finish.cpp)
  add_dependencies(     wait_for_trial_finish  hast_gencpp)
  target_link_libraries(wait_for_trial_finish  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(         count_down  src/count_down.cpp)
  add_dependencies(     count_down  hast_gencpp)
  target_link_libraries(count_down  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(         stereoObs src/stereoObs.cpp)
 add_dependencies(      stereoObs hast_gencpp )
 target_link_libraries( stereoObs utils ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(         uavAutopilot    src/uavAutopilot.cpp)
  add_dependencies(     uavAutopilot    hast_gencpp)
    target_link_libraries(uavAutopilot  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(         ugvAction     src/ugvAction.cpp)
  add_dependencies(     ugvAction     hast_gencpp)
  target_link_libraries(ugvAction     ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(         goalPub     src/goalPub.cpp)
  add_dependencies(     goalPub     hast_gencpp)
  target_link_libraries(goalPub     ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(         goalPubCli  src/goalPubCli.cpp)
  add_dependencies(     goalPubCli  hast_gencpp)
  target_link_libraries(goalPubCli  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# ARC_ALT code
# add_executable(         rostimeCaller  src/rostimeCaller.cpp)
#   add_dependencies(     rostimeCaller  hast_gencpp)
#   target_link_libraries(rostimeCaller  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(         killhast  src_alt/killHast.cpp)
  add_dependencies(     killhast  hast_gencpp)
  target_link_libraries(killhast  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(         shutdownListener   src_alt/shutdownListener.cpp)
  add_dependencies(     shutdownListener   hast_gencpp)
  target_link_libraries(shutdownListener   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(         rawImagePub   src_alt/rawImagePubClass.cpp)
  add_dependencies(     rawImagePub   hast_gencpp)
  target_link_libraries(rawImagePub   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(         triggerSaveRaw   src_alt/triggerSaveRaw.cpp)
  add_dependencies(     triggerSaveRaw   hast_gencpp)
  target_link_libraries(triggerSaveRaw   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(         uavCal          src_alt/uavCal.cpp)
  add_dependencies(     uavCal          hast_gencpp)
  target_link_libraries(uavCal          ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(         uavDownImage    src_alt/uavDownImage.cpp)
  add_dependencies(     uavDownImage    hast_gencpp)
  target_link_libraries(uavDownImage    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(         ugvJockey   src/ugvJockey.cpp)
#  add_dependencies(     ugvJockey   hast_gencpp)
#  target_link_libraries(ugvJockey   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# install(PROGRAMS launch/src_zipper.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(         offlineStereo  src_alt/offlineStereo.cpp)
#  add_dependencies(     offlineStereo  hast_gencpp )
#  target_link_libraries(offlineStereo  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(         offlineStereo2  src_alt/offlineStereo2.cpp)
  #add_dependencies(     offlineStereo2  hast_gencpp )
  #target_link_libraries(offlineStereo2  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable(         jointSLAM_c  src/jointSLAM_c.cpp src/jointSLAM_fgen.cpp src/jointSLAM_ftags.cpp)
#   add_dependencies(     jointSLAM_c  hast_gencpp)
#   target_link_libraries(jointSLAM_c  utils fileops hastCKF hastDKF  hastUAV hastUGV hastAPRIL ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

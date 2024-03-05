cmake_minimum_required(VERSION 3.0.2)
project(rbkairos_webots)


find_package(catkin REQUIRED COMPONENTS
    # roscpp
    # std_srvs
    # std_msgs
    # sensor_msgs
    # tf
    # joint_state_controller
    # velocity_controllers
		# robotnik_msgs
    # webots_ros
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
CATKIN_DEPENDS roscpp sensor_msgs
)


#############
## Install ##
#############


install(
  DIRECTORY launch worlds rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


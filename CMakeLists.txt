################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(karmasim_dev_pkg)


################################################################################
# Find python files
################################################################################
file(GLOB python_scripts_DIR
  "src/*.py"
)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  karmasim_ros_wrapper
)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
        FILES
        SampleMessage.msg
)
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)
################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS message_runtime rospy geometry_msgs karmasim_ros_wrapper
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################


catkin_install_python(PROGRAMS ${python_scripts_DIR}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################

cmake_minimum_required(VERSION 2.8.3)
project(scenenet_ros_tools)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

############
# BINARIES #
############

catkin_install_python(PROGRAMS pointcloud_publisher.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

##########
# EXPORT #
##########
cs_install()
cs_export()

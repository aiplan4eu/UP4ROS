cmake_minimum_required(VERSION 3.0.4)
project(up4ros)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

#catkin_install_python(
#    PROGRAMS
#     scripts/up4ros.py
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
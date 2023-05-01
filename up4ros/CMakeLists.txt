cmake_minimum_required(VERSION 3.0.4)
project(up4ros)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(
    PROGRAMS
    scripts/start_up4ros.py
    scripts/example_client.py
    scripts/example_pddl_client.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY tests/pddl/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pddl)


if (CATKIN_ENABLE_TESTING)
    find_package(ros_pytest REQUIRED)
    add_pytests(tests)
endif()
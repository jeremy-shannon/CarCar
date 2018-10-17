cmake_minimum_required(VERSION 2.8.3)
project(record_path_node)

find_package(catkin REQUIRED COMPONENTS roslint)

catkin_python_setup()
catkin_package()

install(PROGRAMS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Check package for pep8 style, add a test to fail on violations.
roslint_python()
roslint_add_test()

cmake_minimum_required(VERSION 2.8.3)
project(harcar_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs std_msgs)
# We want boost/format.hpp, which isn't in its own component.
#find_package(Boost REQUIRED)

add_message_files(FILES Path.msg CarControl.msg)
generate_messages(
	DEPENDENCIES 
	geometry_msgs
	std_msgs
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

catkin_package(
#  CATKIN_DEPENDS message_runtime std_msgs
#  CATKIN_DEPENDS message_runtime geometry_msgs
)

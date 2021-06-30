# cmake_minimum_required(VERSION 3.0.2)
# project(drivetrain)

# ## Compile as C++11, supported in ROS Kinetic and newer
# # add_compile_options(-std=c++11)

# ## Find catkin macros and libraries
# ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# ## is used, also find other catkin packages


# ## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Motor.msg
# )
# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES drivetrain
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime
# #  DEPENDS system_lib
# )

# ###########
# ## Build ##
# ###########

# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# include_directories(
# # include
#   ${catkin_INCLUDE_DIRS}
# )
# add_executable(set_speed src/set_speed.cpp)
# target_link_libraries(set_speed ${catkin_LIBRARIES})


project(drivetrain)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation)
## Declare ROS messages and services
add_message_files(FILES Motor.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS rospy roscpp message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(speed_control src/speed_control.cpp)
add_dependencies(speed_control drivetrain_generate_messages_cpp)
target_link_libraries(speed_control ${catkin_LIBRARIES})

add_executable(wheel_control src/wheel_control.cpp)
target_link_libraries(wheel_control ${catkin_LIBRARIES})
add_dependencies(wheel_control drivetrain_generate_messages_cpp)

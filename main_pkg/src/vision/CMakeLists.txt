project(vision)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pantilt src/pantilt.cpp)
target_link_libraries(pantilt ${catkin_LIBRARIES})
# add_dependencies(vision beginner_tutorials_generate_messages_cpp)

# What version of CMake is needed?
cmake_minimum_required(VERSION 3.0.2)

# Name of this package.
project(qr_vision)

# Find the catkin build system, and any other packages on
# which we depend.
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs tf2 message_generation)

# Configure python related settings for this package.
catkin_python_setup()

add_message_files(
  FILES
  msg/Marker.msg
  msg/MarkerArray.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

# Declare our catkin package.
catkin_package()

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files. If
# there are multiple executables, use multiple copies of
# this line.
add_executable(publisher src/publisher.cpp)

# Specify libraries against which to link. Again, this
# line should be copied for each distinct executable in
# the package
target_link_libraries(publisher ${catkin_LIBRARIES})
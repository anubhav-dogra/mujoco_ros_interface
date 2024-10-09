cmake_minimum_required(VERSION 3.0.2)
project(mujoco_ros_interface)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp                                                # ROS1 equivalent to rclcpp
  sensor_msgs                                           # sensor_msgs::JointState
  std_msgs                                              # std_msgs::Float64MultiArray
  controller_manager                                    # controller_manager::ControllerManager
)

find_package(glfw3 REQUIRED)                            # Graphics Library FrameWork

# MuJoCo
set(MUJOCO_PATH "/home/terabotics/mujoco_ws/src/mujoco/mujoco-3.2.3")             # UPDATE THIS TO YOUR MUJOCO PATH
include_directories(include                           # Local header files
            ${catkin_INCLUDE_DIRS}
            ${MUJOCO_PATH}/include/mujoco               # MuJoCo header files
            ${MUJOCO_PATH}/include                      # Additional MuJoCo header files
)
link_directories(${MUJOCO_PATH}/lib)                    # Location of MuJoCo libraries

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)
# Build the library (hardware interface)
add_library(mujoco_hw_interface
             src/mujoco_hw_interface.cpp)

# Build the executable
add_executable(mujoco_interface_node src/mujoco_interface_node.cpp
                                     src/MujocoInterface.cpp)

add_executable(iiwa src/mujoco_hw_interface_node.cpp)

                                     
target_link_libraries(mujoco_hw_interface ${catkin_LIBRARIES})
target_link_libraries(mujoco_interface_node mujoco glfw ${catkin_LIBRARIES})
target_link_libraries(iiwa mujoco_hw_interface ${catkin_LIBRARIES})

add_dependencies(mujoco_hw_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# Specify additional locations of header files
include_directories(
  include 
  ${catkin_INCLUDE_DIRS})

# Install rules
# install(TARGETS mujoco_interface_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# install(DIRECTORY config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

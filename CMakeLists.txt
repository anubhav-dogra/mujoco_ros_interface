cmake_minimum_required(VERSION 3.5)
project(mujoco_ros_interface)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(controller_manager REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(glfw3 REQUIRED) # Graphics Library Framework
# MuJoCo
set(MUJOCO_PATH "/opt/mujoco/mujoco-3.2.6")             # UPDATE THIS TO YOUR MUJOCO PATH
include_directories(
            include                           # Local header files
            ${MUJOCO_PATH}/include/mujoco               # MuJoCo header files
            ${MUJOCO_PATH}/include                      # Additional MuJoCo header files
)
link_directories(${MUJOCO_PATH}/lib)                    # Location of MuJoCo libraries

# Build the library (hardware interface)
# add_library(mujoco_hw_interface
#              src/mujoco_hw_interface.cpp)

# Build the executable
add_executable(mujoco_interface_node src/mujoco_interface_node.cpp
                                     src/MujocoInterface.cpp)

# add_executable(iiwa src/mujoco_hw_interface_node.cpp)
                          
# target_link_libraries(mujoco_hw_interface ${catkin_LIBRARIES})
target_link_libraries(mujoco_interface_node mujoco glfw)
# target_link_libraries(iiwa mujoco_hw_interface ${catkin_LIBRARIES})

ament_target_dependencies(mujoco_interface_node
  rclcpp
  sensor_msgs
  std_msgs
  # controller_manager
  tf2_ros
  glfw3)
# Install targets

install(TARGETS
   mujoco_interface_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()

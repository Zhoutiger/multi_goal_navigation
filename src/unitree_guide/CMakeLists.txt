cmake_minimum_required(VERSION 3.10)
project(unitree_guide)

add_definitions(-DCOMPILE_WITH_ROS)
#add_definitions(-DCOMPILE_DEBUG)
#add_definitions(-DCOMPILE_WITH_SLAM)

# set(CMAKE_CXX_FLAGS "-O3")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    std_msgs
    tf
    geometry_msgs
    unitree_legged_msgs
    message_generation
)


add_message_files(
  FILES
  Command.msg
  # Message2.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

#find_package(Python2 COMPONENTS Interpreter Development NumPy)

find_package(gazebo REQUIRED)

catkin_package(
    CATKIN_DEPENDS 
    message_generation
    unitree_legged_msgs 
    message_runtime
)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
    library/unitree_legged_sdk/lib
    ${GAZEBO_LIBRARY_DIRS}
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3")

# aux_source_directory(src SRC_LIST)
file(GLOB_RECURSE SRC_LIST
"src/*/*.cpp"
"src/*/*.cc"
)

add_executable(junior_ctrl src/main.cpp ${SRC_LIST})
target_link_libraries(junior_ctrl ${catkin_LIBRARIES})
add_dependencies(junior_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_include_directories(junior_ctrl PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
#target_link_libraries(junior_ctrl Python2::Python Python2::NumPy)
target_link_libraries(junior_ctrl libunitree_legged_sdk_amd64.so)
target_link_libraries(junior_ctrl -pthread lcm)
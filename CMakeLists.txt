cmake_minimum_required(VERSION 2.8.3)
project(lwm)

find_package(catkin REQUIRED COMPONENTS
    geolib2
)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
# )

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(lwm
    include/lwm/world_model.h
    include/lwm/ros/lrf_buffer.h
    include/lwm/visualization/viewer.h

    src/world_model.cpp
    src/ros/lrf_buffer.cpp
    src/visualization/viewer.cpp
)
target_link_libraries(lwm ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

add_executable(test_lrf test/test_lrf.cpp)
target_link_libraries(test_lrf lwm)



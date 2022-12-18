cmake_minimum_required(VERSION 2.8.3)
project(gnss_observations)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs
  nav_msgs
  sensor_msgs
  cmake_modules
  message_generation
)

find_package(Eigen3 REQUIRED)

add_message_files(FILES 
        Observation.msg
        EpochData.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

## Setup include directories
include_directories(
	${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
	include/
  library/
)

LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/library)

catkin_package(
  INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp std_msgs message_runtime nav_msgs
)

set (CMAKE_CXX_FLAGS "-fpermissive")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -O3 -ansi -pedantic -Wno-unused-but-set-variable")

add_definitions (
  -DENAGLO -DENAQZS -DENACMP -DENAGAL -DNFREQ=3 -DTRACE -DDLL
)

## Create the nodelet tutorial library
# ---------------- gnss data ---------------
aux_source_directory(src/rtklib DIR_SRCS_RTKLIB_1)
aux_source_directory(src/rtklib/src DIR_SRCS_RTKLIB_2)
aux_source_directory(src/rtklib/src/rcv DIR_SRCS_RTKLIB_3)

add_executable(gnss_data src/gnss_data.cpp 
    ${DIR_SRCS_RTKLIB_1} ${DIR_SRCS_RTKLIB_2} ${DIR_SRCS_RTKLIB_3})

add_dependencies(gnss_data 
       ${${PROJECT_NAME}_EXPORTTED_TARGETS}
       ${catkin_EXPORTED_TARGETS}
       ${PROJECT_NAME}_generate_messages_cpp
       ${PROJECT_NAME}_gencpp
       )

target_link_libraries(gnss_data 
  ${catkin_LIBRARIES} -lm -lrt -lpthread)



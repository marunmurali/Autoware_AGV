cmake_minimum_required(VERSION 2.8.3)
project(lc_processor)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        sensor_msgs
        pcl_conversions
        message_generation
        autoware_config_msgs
        )


generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        pcl_ros
        sensor_msgs
        pcl_conversions
	message_generation
	autoware_config_msgs
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})
SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
add_compile_options(-std=c++11)

add_executable(lc_processor src/lc_processor.cpp)
add_dependencies(lc_processor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lc_processor ${catkin_LIBRARIES})


install(TARGETS
            lc_processor
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

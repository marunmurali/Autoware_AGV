cmake_minimum_required(VERSION 2.8.3)
project(jude_percept)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        sensor_msgs
        pcl_conversions
        message_generation
        autoware_config_msgs
        autoware_msgs
        geometry_msgs
        std_msgs
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
        autoware_msgs
	geometry_msgs
	std_msgs
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})
SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
add_compile_options(-std=c++11)

## Realtime LC filter
add_executable(realtime_lc_filter nodes/realtime_lc_filter/realtime_lc_filter.cpp)

add_dependencies(realtime_lc_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(realtime_lc_filter ${catkin_LIBRARIES})

## Grid Filter 
add_executable(grid_filter nodes/grid_filter/grid_filter.cpp)

add_dependencies(grid_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(grid_filter ${catkin_LIBRARIES})

## Lane Change monitor 
add_executable(lane_change_monitor nodes/lane_change_monitor/lane_change_monitor.cpp)

add_dependencies(lane_change_monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(lane_change_monitor ${catkin_LIBRARIES})


## Grid_detector 
add_executable(grid_detector nodes/grid_detector/grid_detector.cpp)

add_dependencies(grid_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(grid_detector ${catkin_LIBRARIES})


install(TARGETS 
	    realtime_lc_filter
	    grid_filter
            lane_change_monitor
            grid_detector
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

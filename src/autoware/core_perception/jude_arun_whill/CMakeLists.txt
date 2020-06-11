cmake_minimum_required(VERSION 2.8.3)
project(jude_arun_whill)



#読み込むパッケージ
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  autoware_config_msgs
  std_msgs
  autoware_msgs
  tf
  tf2
  tf2_geometry_msgs
  serial
  genmsg
  pcl_ros
  geometry_msgs 
  sensor_msgs
  pcl_conversions 
  dynamic_reconfigure
)

#使用するメッセージファイル
add_message_files(FILES
  csm_data.msg
  csm_status.msg
  whill_data.msg
  Anchor.msg
  Tag.msg

)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Anchor_0.srv
   Anchor_1.srv
   Anchor_2.srv
   Anchor_3.srv
   Tag_srv.srv
#   Service2.srv
 )

## Generate messages 
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    cfg/DWM1001_Tune_Serial.cfg
)


#インクルード
catkin_package(
  INCLUDE_DIRS
  LIBRARIES whill
  CATKIN_DEPENDS message_runtime roscpp std_msgs tf tf2 tf2_geometry_msgs serial sensor_msgs geometry_msgs autoware_config_msgs autoware_msgs message_generation pcl_ros pcl_conversions dynamic_reconfigure

)


## Install for python programs 
catkin_install_python(PROGRAMS
        nodes/whill/WHILL_Command_Model_A.py
        nodes/whill/WHILL_Command_Model_C.py
        nodes/whill/whill_node_Model_A.py
        nodes/whill/whill_node_Model_C.py
        nodes/dwm1001/dwm1001_main.py
	nodes/dwm1001/dwm1001_apiCommands.py
	nodes/dwm1001/dwm1001_systemDefinitions.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})
add_compile_options(-std=c++11)
add_compile_options(-Wall -Wextra)


## whill node 
add_executable(cmd_vel_multiplexer nodes/whill/cmd_vel_multiplexer.cpp)
add_dependencies(cmd_vel_multiplexer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_vel_multiplexer ${catkin_LIBRARIES}
)

## Joystick teleop node
add_executable(joystick_teleop_node nodes/joystick_teleop/joystick_teleop_node.cpp)
add_dependencies(joystick_teleop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joystick_teleop_node ${catkin_LIBRARIES})

## Build autoware twist converter 
add_executable(autoware_twist_converter nodes/autoware_twist_converter/autoware_twist_converter.cpp)
add_dependencies(autoware_twist_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(autoware_twist_converter ${catkin_LIBRARIES})

## Build obstacle stop
add_executable(obstacle_stop nodes/obstacle_stop/obstacle_stop.cpp)
add_dependencies(obstacle_stop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_stop ${catkin_LIBRARIES})

## Build combine gps
add_executable(combine_gps nodes/combine_gps/combine_gps.cpp)
add_dependencies(combine_gps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(combine_gps ${catkin_LIBRARIES})

## Build pose predictor
add_executable(pose_predictor nodes/pose_predictor/pose_predictor.cpp)
add_dependencies(pose_predictor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_predictor ${catkin_LIBRARIES})

## Build dwm1001
# add_executable(dwm1001 nodes/dwm1001/dwm1001_main.py)
# add_dependencies(dwm1001 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(dwm1001 ${catkin_LIBRARIES})



install(TARGETS 
	    cmd_vel_multiplexer
	    joystick_teleop_node
            autoware_twist_converter
            obstacle_stop
	    combine_gps
	    pose_predictor
	    # dwm1001
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
find_package(rostest REQUIRED)

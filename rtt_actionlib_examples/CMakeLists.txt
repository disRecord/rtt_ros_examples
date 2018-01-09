cmake_minimum_required(VERSION 2.8.3)
project(rtt_actionlib_examples)

### c++11 support ###
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp rtt_ros actionlib actionlib_msgs genmsg rtt_roscomm)

add_action_files(DIRECTORY action FILES SomeAction.action)
generate_messages(DEPENDENCIES actionlib_msgs)

ros_generate_rtt_typekit(${PROJECT_NAME})

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS})

orocos_use_package(ocl-deployment REQUIRED)
orocos_use_package(ocl-taskbrowser REQUIRED)

orocos_executable(test_server test_server.cpp)
target_link_libraries(
  test_server 
  ${catkin_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${USE_OROCOS_LIBRARIES}
  )
add_dependencies(test_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

orocos_executable(test_server_simple test_server_simple.cpp)
target_link_libraries(
  test_server_simple
  ${catkin_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${USE_OROCOS_LIBRARIES}
  )
add_dependencies(test_server_simple ${${PROJECT_NAME}_EXPORTED_TARGETS})

orocos_executable(test_server_simple_poll test_server_simple_poll.cpp)
target_link_libraries(
  test_server_simple_poll
  ${catkin_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${USE_OROCOS_LIBRARIES}
  )
add_dependencies(test_server_simple_poll ${${PROJECT_NAME}_EXPORTED_TARGETS})

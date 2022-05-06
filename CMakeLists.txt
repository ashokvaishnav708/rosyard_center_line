cmake_minimum_required(VERSION 2.8.3)
project(rosyard_estimation)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    std_msgs
    gazebo_msgs
    rosyard_common
    tf
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  DEPENDS rosyard_common
)


# add python nodes here (REMINDER: make py-file executable)
catkin_install_python(PROGRAMS
        scripts/center_line_estimation_py.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${roscpp_INCLUDE_DIRS}
)

add_executable(velocity src/velocity_node.cpp src/velocity_estimator.cpp)
add_dependencies(velocity ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity PRIVATE ${catkin_LIBRARIES})
target_compile_features(velocity PRIVATE cxx_auto_type cxx_raw_string_literals cxx_user_literals cxx_generic_lambdas)


add_executable(center_line_estimator src/center_line_estimator.cpp)
add_dependencies(center_line_estimator ${catkin_EXPORTED_TARGETS})
target_link_libraries(center_line_estimator PRIVATE ${catkin_LIBRARIES})
target_compile_features(center_line_estimator PRIVATE cxx_auto_type cxx_raw_string_literals cxx_user_literals cxx_generic_lambdas)


add_executable(lap_status_estimator src/lap_status_estimator.cpp)
add_dependencies(lap_status_estimator ${catkin_EXPORTED_TARGETS})
target_link_libraries(lap_status_estimator PRIVATE ${catkin_LIBRARIES})
target_compile_features(lap_status_estimator PRIVATE cxx_auto_type cxx_raw_string_literals cxx_user_literals cxx_generic_lambdas)

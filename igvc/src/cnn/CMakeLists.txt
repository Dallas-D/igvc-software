add_executable(nn_reader main.cpp)
add_dependencies(nn_reader igvc_msgs_gencpp)
target_link_libraries(nn_reader ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

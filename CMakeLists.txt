cmake_minimum_required(VERSION 2.8.3)
project(sikil4_io)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg)

add_message_files(DIRECTORY msg FILES adc.msg pwm.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS roscpp std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/sikil4_io_node.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${wiringPi_LIB})


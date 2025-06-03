cmake_minimum_required(VERSION 3.14.4)
project(swiftpro)




if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()




find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SwiftproState.msg"
  "msg/Position.msg"
  "msg/Angle4th.msg"
  "msg/Status.msg"
  DEPENDENCIES std_msgs
)


rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")




add_executable(swiftpro_write_node src/swiftpro_write_node.cpp)
rosidl_target_interfaces(swiftpro_write_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_include_directories(swiftpro_write_node PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(swiftpro_write_node rclcpp std_msgs sensor_msgs tf2 tf2_ros serial tf2_geometry_msgs)
target_link_libraries(swiftpro_write_node ${cpp_typesupport_target})

add_executable(swiftpro_moveit_node src/swiftpro_moveit_node.cpp)
rosidl_target_interfaces(swiftpro_moveit_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_include_directories(swiftpro_moveit_node  PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(swiftpro_moveit_node rclcpp std_msgs sensor_msgs tf2 tf2_ros serial tf2_geometry_msgs)
target_link_libraries(swiftpro_moveit_node ${cpp_typesupport_target})

add_executable(swiftpro_rviz_node src/swiftpro_rviz_node.cpp)
rosidl_target_interfaces(swiftpro_rviz_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_include_directories(swiftpro_rviz_node PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(swiftpro_rviz_node rclcpp std_msgs sensor_msgs tf2 tf2_ros serial tf2_geometry_msgs)
target_link_libraries(swiftpro_rviz_node ${cpp_typesupport_target})

add_executable(swiftpro_read_node src/swiftpro_read_node.cpp)
rosidl_target_interfaces(swiftpro_read_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_include_directories(swiftpro_read_node PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(swiftpro_read_node rclcpp std_msgs sensor_msgs tf2 tf2_ros serial tf2_geometry_msgs)
target_link_libraries(swiftpro_read_node ${cpp_typesupport_target})


target_link_libraries(swiftpro_write_node rclcpp::rclcpp ${std_msgs_TARGETS})
target_link_libraries(swiftpro_moveit_node rclcpp::rclcpp ${std_msgs_TARGETS})
target_link_libraries(swiftpro_rviz_node rclcpp::rclcpp ${std_msgs_TARGETS})
target_link_libraries(swiftpro_read_node rclcpp::rclcpp ${std_msgs_TARGETS})

target_link_libraries(swiftpro_write_node serial)


install(TARGETS
	swiftpro_write_node
	swiftpro_moveit_node
	swiftpro_rviz_node
	swiftpro_read_node
	DESTINATION lib/${PROJECT_NAME}
 )


ament_export_dependencies(rosidl_default_runtime)

ament_package()



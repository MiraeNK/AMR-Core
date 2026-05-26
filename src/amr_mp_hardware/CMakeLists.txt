
cmake_minimum_required(VERSION 3.5)

project(amr_mp_hardware)



if(NOT CMAKE_CXX_STANDARD)

  set(CMAKE_CXX_STANDARD 14)

endif()



if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  add_compile_options(-Wall -Wextra -Wpedantic)

endif()



find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(hardware_interface REQUIRED)

find_package(pluginlib REQUIRED)

find_package(std_msgs REQUIRED)



add_library(${PROJECT_NAME} SHARED

  src/polebot_hardware_interface.cpp

)



target_include_directories(${PROJECT_NAME}

  PRIVATE include

)



ament_target_dependencies(${PROJECT_NAME}

  rclcpp

  hardware_interface

  pluginlib

  std_msgs

)



pluginlib_export_plugin_description_file(

  hardware_interface amr_mp_hardware_plugin.xml)



install(TARGETS ${PROJECT_NAME}

  ARCHIVE DESTINATION lib

  LIBRARY DESTINATION lib

  RUNTIME DESTINATION bin

)



install(DIRECTORY include/

  DESTINATION include

)



install(FILES amr_mp_hardware_plugin.xml

  DESTINATION share/${PROJECT_NAME}

)



ament_package()


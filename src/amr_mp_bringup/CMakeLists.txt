
cmake_minimum_required(VERSION 3.5)

project(amr_mp_bringup)

find_package(ament_cmake REQUIRED)

install(

  DIRECTORY config launch

  DESTINATION share/${PROJECT_NAME}

)

ament_package()


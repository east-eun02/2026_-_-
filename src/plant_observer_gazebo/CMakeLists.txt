cmake_minimum_required(VERSION 3.8)
project(plant_observer_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds launch config rviz DESTINATION share/${PROJECT_NAME})

ament_package()

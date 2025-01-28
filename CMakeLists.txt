cmake_minimum_required(VERSION 3.0.2)
project(lsuwb)

## Encontrar pacotes necessários
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

## Configurar o pacote
catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs
)

## Incluir diretórios
include_directories(
  ${catkin_INCLUDE_DIRS}
)
cmake_minimum_required(VERSION 3.16)

# Definir o nome do projeto
project(lpuwb)

# Encontrar pacotes ROS necessários
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

# Adicionar arquivos de mensagem
add_message_files(
  FILES
  Tag.msg
)

# Gerar as mensagens
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Pacote catkin
catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

# Incluir diretórios de dependências
include_directories(
  ${catkin_INCLUDE_DIRS}
)

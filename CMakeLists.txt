cmake_minimum_required(VERSION 2.8.3)
project(sentor)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  TopicMap.msg
  TopicMapArray.msg
)

add_service_files(
  FILES
  GetTopicMaps.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

install(PROGRAMS
  scripts/sentor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach (dir launch config)
    install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

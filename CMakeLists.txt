cmake_minimum_required(VERSION 2.8.3)
project(sentor)
find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/sentor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach (dir launch config)
    install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

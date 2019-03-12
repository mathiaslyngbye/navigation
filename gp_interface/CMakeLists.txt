cmake_minimum_required(VERSION 2.8.3)
project(gp_interface)

find_package(catkin REQUIRED
    COMPONENTS
        angles
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        nav_core
        navfn
        nav_msgs
        pluginlib
        roscpp
        tf
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        nav_core
        navfn
        nav_msgs
        pluginlib
        roscpp
        tf
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
    src/gp_interface.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS gp_interface
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



cmake_minimum_required(VERSION 2.8.3)
project(ras_group8_localization)




find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  phidgets
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
    include

  CATKIN_DEPENDS
    roscpp
    phidgets
    std_msgs
    geometry_msgs
    sensor_msgs

# DEPENDS
)

find_package(PkgConfig)

pkg_check_modules(BFL REQUIRED orocos-bfl)
if(BFL_FOUND)
  message("find state")
endif(BFL_FOUND)
include_directories(${BFL_INCLUDE_DIRS}/bfl)
message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
message("BFL library dirs:" ${BFL_LIBRARY_DIRS})
link_directories(${BFL_LIBRARY_DIRS})


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/Localization.cpp
  src/customparticlefilter.cpp
  src/nonlinearMeasurementPdf.cpp
  src/nonlinearSysetmPdf.cpp

)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME} 
  ${BFL_LIBRARIES}
)

catkin_add_gtest(localization_tests test/test_localization.cpp src/Localization.cpp)

# Make main executable
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)






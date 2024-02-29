cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar)
#set(CMAKE_VERBOSE_MAKEFILE ON)

## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(Eigen3 REQUIRED)
if(NOT Eigen3_FOUND)
    message(FATAL_ERROR "\n Eigen 3 NOT FOUND \n")
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
    geometry_msgs
    message_generation
    nav_msgs
)

#for map parser
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
find_package(Bullet REQUIRED)


#add_service_files(DIRECTORY srv)

add_service_files(
    FILES
    addCubesSrv.srv
    cubeReqSrv.srv
    planReqSrv.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
)



set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node2d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node3d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collisiondetection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/planner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/path.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/smoother.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/visualize.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dubins.cpp #Andrew Walker
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamicvoronoi.cpp #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bucketedqueue.cpp #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cube_block.cpp
    )
set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/algorithm.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/node2d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/node3d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/collisiondetection.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/planner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/path.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/smoother.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vector2d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/visualize.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/helper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/constants.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/lookup.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gradient.h #Andrew Noske
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dubins.h #Andrew Walker
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dynamicvoronoi.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bucketedqueue.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/point.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cube_block.h
    )
add_library(HYAS ${SOURCES} ${HEADERS})

## Declare a catkin package
catkin_package(CATKIN_DEPENDS rospy std_msgs geometry_msgs
    LIBRARIES
        map_server_image_loader)

## OPEN MOTION PLANNING LIBRARY
find_package(ompl REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${OMPL_INCLUDE_DIRS})
include_directories(include include)
include_directories(include ${YAMLCPP_INCLUDE_DIRS} maps ${SDL_INCLUDE_DIR}
${SDL_IMAGE_INCLUDE_DIRS} ${BULLET_INCLUDE_DIRS})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(hybrid_astar src/main.cpp ${HEADERS} ${SOURCES})
target_link_libraries(hybrid_astar ${catkin_LIBRARIES})
target_link_libraries(hybrid_astar ${OMPL_LIBRARIES})
target_link_libraries(hybrid_astar yaml-cpp ${Boost_INCLUDE_DIRS} ${SDL_LIBRARY}
${SDL_IMAGE_LIBRARIES} ${BULLET_LIBRARIES})

add_executable(draw_cube src/draw_cube.cpp)
target_link_libraries(draw_cube ${catkin_LIBRARIES})
#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} msg_pkg_generate_messages_cpp)


install(TARGETS ${PROJECT_NAME} tf_broadcaster
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
scripts/test_plan_client.py  # Replace with the actual name of your Python script
scripts/test_task_allocation.py
scripts/agent.py
scripts/communications.py
scripts/marker_publisher.py
scripts/task.py
scripts/transformation_tools.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

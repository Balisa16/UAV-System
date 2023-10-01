cmake_minimum_required(VERSION 3.0.2)
project(emiro)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  mavros
  cv_bridge
  image_transport
  std_msgs
  geometry_msgs
  tf
)

find_package(OpenCV REQUIRED)
find_package(JetsonGPIO)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Curses REQUIRED)
find_package(CURL REQUIRED)
find_package(jsoncpp QUIET)

if (NOT jsoncpp_FOUND)
    message(WARNING "Library jsoncpp not found.")
else()
  catkin_package(
    LIBRARIES emiro
    CATKIN_DEPENDS cv_bridge image_transport std_msgs
    DEPENDS system_lib
  )
  
  include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${CURSES_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include)

  add_definitions(${OpenCV_DEFENITIONS})

  # set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

  add_library(${PROJECT_NAME} SHARED
    include/param.hpp
    include/convert.hpp
    include/copter.hpp
    include/enum.hpp
    include/GPSnav.hpp
    include/lidar.hpp
    include/logger.hpp
    include/pin.hpp
    include/rangefinder.hpp
    include/servo.hpp
    include/tfmini.hpp
    include/vision.hpp
    src/param.cc
    src/convert.cc
    src/copter.cc
    src/GPSnav.cc
    src/lidar.cc
    src/logger.cc
    src/pin.cc
    src/servo.cc)

  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/example)

  add_executable(test1 example/test1.cc)
  target_link_libraries(test1 ${PROJECT_NAME} 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    ${CURSES_LIBRARIES}
    JetsonGPIO 
    Boost::system 
    Boost::filesystem)
endif()
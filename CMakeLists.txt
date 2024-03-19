cmake_minimum_required(VERSION 3.10)
project(emiro VERSION 0.1.0)

# set(CMAKE_C_COMPILER "/usr/lib/ccache/clang")
# set(CMAKE_CXX_COMPILER "/usr/lib/ccache/clang++")

# add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  mavros
  cv_bridge
  image_transport
  std_msgs
  geometry_msgs
  tf)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/release/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/release/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/release/lib)

set(emiro_file
  src/control.cc
  src/copter.cc
  src/gps.cc
  src/jsonio.cc
  src/lidar.cc
  src/param.cc
  src/rangefinder.cc
  src/servo.cc
  src/simplepid.cc
  src/tfmini.cc
  src/uart.cc)


# Check Platform
set(IS_JETSON OFF CACHE BOOL "Is Jetson platform")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "AARCH64 architecture detected")
  set(IS_JETSON ON)
  add_subdirectory(thirdparty/JetsonGPIO)
  else()
  message(STATUS "Not an AARCH64 architecture")
endif()


find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Curses REQUIRED)
find_package(CURL REQUIRED)
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(LIBNOTIFY REQUIRED libnotify)

add_subdirectory(thirdparty/Logger)
add_subdirectory(thirdparty/jsoncpp)
add_subdirectory(thirdparty/asynccamera)

set(emiro_include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Logger/include
  ${catkin_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/jsoncpp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/asynccamera/include)

if(IS_JETSON)
  message(STATUS "Adding JetsonGPIO in library")
  add_library(${PROJECT_NAME} SHARED
    ${emiro_file}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pin.cc)

  target_include_directories(${PROJECT_NAME} PUBLIC
    ${emiro_include}
    ${CMAKE_CURRENT_SOURCE_DIR}/platform)

  target_link_libraries(${PROJECT_NAME} PRIVATE
    Logger
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS} 
    ${CURSES_LIBRARIES}
    Boost::system
    asynccamera
    Boost::filesystem
    jsoncpp
    JetsonGPIO)

else()
  message(STATUS "Not adding Jetson GPIO in library")
  add_library(${PROJECT_NAME} SHARED
    ${emiro_file})

  target_include_directories(${PROJECT_NAME} PUBLIC
    ${emiro_include})

  target_link_libraries(${PROJECT_NAME} PRIVATE
    Logger
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS} 
    ${CURSES_LIBRARIES}
    Boost::system
    asynccamera
    Boost::filesystem
    jsoncpp)
endif()

link_directories(${PROJECT_NAME} PUBLIC /usr/local/lib)  

message("")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/release/bin)

add_subdirectory(example)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/release/lib DESTINATION ${CATKIN_DEVEL_PREFIX}/lib)
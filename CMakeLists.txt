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

add_subdirectory(third-party/Logger)

# Check Platform
set(IS_JETSON OFF CACHE BOOL "Is Jetson platform")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  message("ARM architecture detected")

  try_compile(IS_JETSON
      ${CMAKE_BINARY_DIR}/pin
      SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/pin.hpp.cpp
      OUTPUT_VARIABLE OUTPUT_VAR
  )

  if(IS_JETSON)
      message("Jetson platform detected")
      add_definitions(-DJETSON)
  else()
      message("Not a Jetson platform")
  endif()
else()
    message("Not an ARM architecture")
endif()


find_package(OpenCV REQUIRED)
if(IS_JETSON)
  message("Find Package JetsonGPIO")
  find_package(JetsonGPIO)
endif()

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Curses REQUIRED)
find_package(CURL REQUIRED)
find_package(jsoncpp REQUIRED)

if (NOT jsoncpp_FOUND)
    message(WARNING "Library jsoncpp not found.")
else()
  # catkin_package(
  #   LIBRARIES emiro
  #   CATKIN_DEPENDS cv_bridge image_transport std_msgs
  #   DEPENDS system_lib
  # )
  
  include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third-party/Logger/include
    ${catkin_INCLUDE_DIRS}
    ${CURSES_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    /usr/local/include
    ${PROJECT_SOURCE_DIR}/include)

  if(IS_JETSON)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/platform/jetson)
  endif()

  link_directories(/usr/local/lib)

  add_definitions(${OpenCV_DEFENITIONS})

  # set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

  if(IS_JETSON)
    message("Adding JetsonGPIO in library")
    add_library(
      ${PROJECT_NAME} SHARED
      src/pin.cc
    )
  endif()

  add_library(${PROJECT_NAME} SHARED
    # include/param.hpp
    # include/convert.hpp
    # include/copter.hpp
    # include/enum.hpp
    # include/GPSnav.hpp
    # include/lidar.hpp
    # include/jsonread.hpp
    # # include/logger.hpp
    # include/pin.hpp
    # include/rangefinder.hpp
    # include/servo.hpp
    # include/tfmini.hpp
    # include/vision.hpp
    src/param.cc
    src/convert.cc
    src/copter.cc
    src/GPSnav.cc
    src/lidar.cc
    src/jsonread.cc
    # src/logger.cc
    src/servo.cc)

  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/example)

  add_executable(test2 example/test1.cpp)
  
  target_link_libraries(test2 ${PROJECT_NAME}
    Logger
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    ${CURSES_LIBRARIES}
    JetsonGPIO 
    Boost::system 
    Boost::filesystem
    jsoncpp
    $<$<BOOL:${IS_JETSON}>:IS_JETSON_PLATFORM)

  set_target_properties(test1 PROPERTIES 
    INSTALL_RPATH "/usr/local/lib"
    BUILD_WITH_INSTALL_RPATH ON
  )

endif()
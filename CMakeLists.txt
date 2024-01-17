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

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/release/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/release/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/release/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/release/bin)

add_subdirectory(thirdparty/Logger)

# Check Platform
set(IS_JETSON OFF CACHE BOOL "Is Jetson platform")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  message("ARM architecture detected")

  try_compile(IS_JETSON
    ${CMAKE_BINARY_DIR}/pin
    SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/platform/jetson/pin.hpp
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
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBNOTIFY REQUIRED libnotify)

add_subdirectory(thirdparty/jsoncpp)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Logger/include
  ${catkin_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  thirdparty/jsoncpp/include
  /usr/local/include
  ${LIBNOTIFY_INCLUDE_DIRS}
)

if(IS_JETSON)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/platform/jetson)
endif()

link_directories(/usr/local/lib)

add_definitions(${OpenCV_DEFENITIONS})

if(IS_JETSON)
  message("Adding JetsonGPIO in library")
  add_library(
    ${PROJECT_NAME} SHARED
    src/pin.cc
  )
endif()

add_library(${PROJECT_NAME} SHARED
  src/param.cc
  src/convert.cc
  src/copter.cc
  src/emiro.cc
  src/lidar.cc
  src/uart.cc
  src/servo.cc)

# message(STATUS "VARIABLES: ${CATKIN_GLOBAL_LIB_DESTINATION}")
# message(STATUS "VARIABLES: ${CATKIN_DEVEL_PREFIX}")

# install(TARGETS ${PROJECT_NAME}
#   LIBRARY DESTINATION ${CATKIN_DEVEL_PREFIX}/${PROJECT_NAME}/lib
# )

# install(TARGETS ${PROJECT_NAME}
#   LIBRARY DESTINATION "/usr/lib")


target_link_libraries(${PROJECT_NAME} PRIVATE
        Logger
        ${catkin_LIBRARIES} 
        ${OpenCV_LIBRARIES} 
        ${CURSES_LIBRARIES}
        Boost::system 
        Boost::filesystem
        jsoncpp
        ${LIBNOTIFY_LIBRARIES}
        $<$<BOOL:${IS_JETSON}>:IS_JETSON_PLATFORM>)

message("")

file(GLOB SRC_FILES example/*.cpp)
foreach(SRC_FILE ${SRC_FILES})
    get_filename_component(TARGET_NAME ${SRC_FILE} NAME_WE)

    message("\tFound : ${TARGET_NAME}")
    
    add_executable(${TARGET_NAME} ${SRC_FILE})

    if(IS_JETSON)
      # message("Adding JetsonGPIO in executable")
      target_link_libraries(${TARGET_NAME} PRIVATE
        ${PROJECT_NAME}
        JetsonGPIO
        Logger
        ${catkin_LIBRARIES} 
        ${OpenCV_LIBRARIES} 
        ${CURSES_LIBRARIES}
        Boost::system 
        Boost::filesystem
        jsoncpp
        ${LIBNOTIFY_LIBRARIES}
        $<$<BOOL:${IS_JETSON}>:IS_JETSON_PLATFORM>)
    else()
      # message("Not a Jetson platform")
      target_link_libraries(${TARGET_NAME} PRIVATE
        ${PROJECT_NAME}
        Logger
        ${catkin_LIBRARIES} 
        ${OpenCV_LIBRARIES} 
        ${CURSES_LIBRARIES}
        Boost::system 
        Boost::filesystem
        jsoncpp
        ${LIBNOTIFY_LIBRARIES}
        $<$<BOOL:${IS_JETSON}>:IS_JETSON_PLATFORM>)
    endif()

    # Hide narrowing warnings
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(${TARGET_NAME} PRIVATE -Wno-narrowing)
    elseif(MSVC)
      target_compile_options(${TARGET_NAME} PRIVATE /wd4838)
    endif()

    
    target_include_directories(${TARGET_NAME} PRIVATE
        thirdparty/glfw/include)

    set_target_properties(${TARGET_NAME} PROPERTIES 
      INSTALL_RPATH "../../install"
      BUILD_WITH_INSTALL_RPATH ON
    )
endforeach()
message("")
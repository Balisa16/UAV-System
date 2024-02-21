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
  tf
)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/release/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/release/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/release/lib)


# Check Platform
set(IS_JETSON OFF CACHE BOOL "Is Jetson platform")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("AARCH64 architecture detected")
  add_subdirectory(thirdparty/JetsonGPIO)
else()
  message("Not an AARCH64 architecture")
endif()


find_package(OpenCV REQUIRED)
if(IS_JETSON)
  message("Find Package JetsonGPIO")
  find_package(JetsonGPIO)
endif()

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Curses REQUIRED)
find_package(CURL REQUIRED)

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(LIBNOTIFY REQUIRED libnotify)

add_subdirectory(thirdparty/Logger)
add_subdirectory(thirdparty/jsoncpp)
add_subdirectory(thirdparty/asynccamera)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Logger/include
  ${catkin_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/jsoncpp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/asynccamera/include
  # /usr/local/include
  # ${LIBNOTIFY_INCLUDE_DIRS}
)
  
if(IS_JETSON)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/platform)
endif()

link_directories(/usr/local/lib)

# add_definitions(${OpenCV_DEFENITIONS})
  
add_library(${PROJECT_NAME} SHARED
  src/control.cc
  # src/convert.cc
  src/copter.cc
  src/gps.cc
  src/jsonio.cc
  src/lidar.cc
  # src/notification.cc
  src/param.cc
  src/rangefinder.cc
  src/servo.cc
  src/simplepid.cc
  src/tfmini.cc
  src/uart.cc
)
  
if(IS_JETSON)
  message("Adding JetsonGPIO in library")
  add_library(${PROJECT_NAME} SHARED
    src/pin.cc
  )
endif()

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
  ${OpenCV_LIBS} 
  ${CURSES_LIBRARIES}
  Boost::system
  asynccamera
  Boost::filesystem
  jsoncpp
  # ${LIBNOTIFY_LIBRARIES}
  $<$<BOOL:${IS_JETSON}>:IS_JETSON_PLATFORM>
)

message("")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/release/bin)

file(GLOB SRC_FILES example/*.cpp)
foreach(SRC_FILE ${SRC_FILES})
  get_filename_component(TARGET_NAME ${SRC_FILE} NAME_WE)

  message("\tFound : ${TARGET_NAME}")

  add_executable(${TARGET_NAME} ${SRC_FILE})

  target_include_directories(${TARGET_NAME} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  
  target_link_libraries(${TARGET_NAME} PUBLIC ${PROJECT_NAME})

  if(IS_JETSON)
    message("Adding JetsonGPIO in executable")
    target_link_libraries(${TARGET_NAME} PRIVATE
      JetsonGPIO
      Logger
      ${catkin_LIBS} 
      ${OpenCV_LIBS} 
      ${CURSES_LIBRARIES}
      Boost::system 
      Boost::filesystem
      asynccamera
      jsoncpp
      # ${LIBNOTIFY_LIBRARIES}
      $<$<BOOL:${IS_JETSON}>:IS_JETSON_PLATFORM>
      ${PROJECT_NAME}
    )
  else()
    target_link_libraries(${TARGET_NAME} PRIVATE
      Logger
      ${catkin_LIBRARIES} 
      ${OpenCV_LIBS} 
      ${CURSES_LIBRARIES}
      Boost::system 
      Boost::filesystem
      asynccamera
      jsoncpp
      # ${LIBNOTIFY_LIBRARIES}
      $<$<BOOL:${IS_JETSON}>:IS_JETSON_PLATFORM>
      ${PROJECT_NAME}
    )
  endif()

  target_compile_options(${TARGET_NAME} PRIVATE 
    -Wno-dev
    -flto 
    -Wunused-variable)
  # Hide narrowing warnings
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${TARGET_NAME} PRIVATE -Wno-narrowing)
  elseif(MSVC)
    target_compile_options(${TARGET_NAME} PRIVATE /wd4838)
  endif()
  
  target_include_directories(${TARGET_NAME} PRIVATE
    thirdparty/glfw/include
    thirdparty/asynccamera/include)

  set_target_properties(${TARGET_NAME} PROPERTIES 
    INSTALL_RPATH "../../install"
    BUILD_WITH_INSTALL_RPATH ON
  )
endforeach()
message("")

# Install library
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/release/lib DESTINATION ${CATKIN_DEVEL_PREFIX}/lib)
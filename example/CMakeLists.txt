file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

message(STATUS "Executable :")
foreach(SRC_FILE ${SRC_FILES})
  get_filename_component(TARGET_NAME ${SRC_FILE} NAME_WE)

  message(STATUS "\t${TARGET_NAME}")

  add_executable(${TARGET_NAME} ${SRC_FILE})

  target_include_directories(${TARGET_NAME} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  
  target_link_libraries(${TARGET_NAME} PUBLIC ${PROJECT_NAME})

  if(IS_JETSON)
    target_link_libraries(${TARGET_NAME} PRIVATE
      JetsonGPIO
      Logger
      ${catkin_LIBRARIES} 
      ${OpenCV_LIBS} 
      ${CURSES_LIBRARIES}
      Boost::system 
      Boost::filesystem
      asynccamera
      jsoncpp
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
      ${PROJECT_NAME}
    )
  endif()

  target_compile_options(${TARGET_NAME} PRIVATE 
    -Wno-dev
    -flto 
    -Wunused-variable)

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
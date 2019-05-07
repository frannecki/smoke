cmake_minimum_required(VERSION 3.5.0)
project(smoke)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(OpenCV_DIR "/home/fran/opencv/build")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
  actionlib_msgs
  actionlib
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)

add_service_files(
  FILES
  smoke.srv
  smoke_svm.srv
  kinect_srv.srv
  darknet_svm_node.srv
)

add_message_files(
  FILES
  alarm.msg
  BoundingBox.msg
  BoundingBoxes.msg
  BboxImage.msg
)

add_action_files(
  FILES
  Alarm.action
)

generate_messages(
  DEPENDENCIES
  smoke
  sensor_msgs
  std_msgs
  actionlib_msgs
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}_lib
    liblbp
  CATKIN_DEPENDS
    roscpp rospy std_msgs actionlib_msgs actionlib sensor_msgs cv_bridge image_transport
  DEPENDS
    Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(THREADS_USE_PTHREADS_WIN32 true)
  find_package(Threads)
  find_package(OpenCV REQUIRED)
  find_package(Boost COMPONENTS thread serialization python REQUIRED)

  if (Threads_FOUND AND Boost_FOUND)
    include_directories(. ${THREADS_PTHREADS_INCLUDE_DIR})
    include_directories(${OpenCV_INCLUDE_DIRS})
    include_directories(${Boost_INCLUDE_DIRS})
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_directories(${OpenCV_LIBRARY_DIR})

    add_library(${PROJECT_NAME}_lib
                ${CMAKE_CURRENT_SOURCE_DIR}/src/rgbData.cpp
                ${CMAKE_CURRENT_SOURCE_DIR}/src/darknetDetector.cpp)

    add_library(liblbp
                ${CMAKE_CURRENT_SOURCE_DIR}/src/LBP.cpp)
    
    target_link_libraries(${PROJECT_NAME}_lib
        freenect
        ${catkin_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        ${MATH_LIB}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
    )

    add_executable(smoke_client ${CMAKE_CURRENT_SOURCE_DIR}/src/smoke_client.cpp)
    add_dependencies(smoke_client smoke_gencpp ${smoke_EXPORTED_TARGETS})
    target_link_libraries(smoke_client ${PROJECT_NAME}_lib)

    add_executable(smoke_svm_client ${CMAKE_CURRENT_SOURCE_DIR}/src/smoke_svm_client.cpp)
    add_dependencies(smoke_svm_client smoke_gencpp ${smoke_EXPORTED_TARGETS})
    target_link_libraries(smoke_svm_client ${PROJECT_NAME}_lib liblbp)

    add_executable(smoke_svm_server ${CMAKE_CURRENT_SOURCE_DIR}/src/smoke_svm_server.cpp)
    add_dependencies(smoke_svm_server smoke_gencpp ${smoke_EXPORTED_TARGETS})
    target_link_libraries(smoke_svm_server ${PROJECT_NAME}_lib liblbp)

    add_executable(darknet_detector ${CMAKE_CURRENT_SOURCE_DIR}/src/darknet_svm_node.cpp)
    add_dependencies(darknet_detector smoke_gencpp ${smoke_EXPORTED_TARGETS})
    target_link_libraries(darknet_detector ${PROJECT_NAME}_lib)

    add_executable(videostr ${CMAKE_CURRENT_SOURCE_DIR}/src/video_stream_node.cpp)
    add_dependencies(videostr smoke_gencpp ${smoke_EXPORTED_TARGETS})
    target_link_libraries(videostr ${catkin_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        ${OpenCV_LIBS})
  endif()

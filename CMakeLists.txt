cmake_minimum_required(VERSION 3.5.0)
project(smoke)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

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
    lbp_lib
    appviz_lib
  CATKIN_DEPENDS
    roscpp rospy std_msgs actionlib_msgs actionlib sensor_msgs cv_bridge image_transport
  DEPENDS
    Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include
)

set(THREADS_USE_PTHREADS_WIN32 true)
  find_package(Threads)
  find_package(OpenCV  REQUIRED)
  find_package(Boost COMPONENTS thread serialization python REQUIRED)
  find_package(Qt5  REQUIRED COMPONENTS  Widgets Core Gui Qml Quick)

  if (Threads_FOUND AND Boost_FOUND)
    include_directories(. ${THREADS_PTHREADS_INCLUDE_DIR})
    include_directories(${OpenCV_INCLUDE_DIRS})
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${OpenCV_LIBRARY_DIR})

    add_library(${PROJECT_NAME}_lib
                ${CMAKE_CURRENT_SOURCE_DIR}/src/rgbData.cpp
                ${CMAKE_CURRENT_SOURCE_DIR}/src/darknetDetector.cpp)

    add_library(lbp_lib
                ${CMAKE_CURRENT_SOURCE_DIR}/src/local_binary_pattern.cpp)
    
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
    target_link_libraries(smoke_svm_client ${PROJECT_NAME}_lib lbp_lib)

    add_executable(smoke_svm_server ${CMAKE_CURRENT_SOURCE_DIR}/src/smoke_svm_server.cpp)
    add_dependencies(smoke_svm_server smoke_gencpp ${smoke_EXPORTED_TARGETS})
    target_link_libraries(smoke_svm_server ${PROJECT_NAME}_lib lbp_lib)

    add_executable(darknet_detector ${CMAKE_CURRENT_SOURCE_DIR}/src/darknet_svm_node.cpp)
    add_dependencies(darknet_detector smoke_gencpp ${smoke_EXPORTED_TARGETS})
    target_link_libraries(darknet_detector ${PROJECT_NAME}_lib)

    add_executable(videostr ${CMAKE_CURRENT_SOURCE_DIR}/src/video_stream_node.cpp)
    add_dependencies(videostr smoke_gencpp ${smoke_EXPORTED_TARGETS})
    target_link_libraries(videostr ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS})
    
    add_executable(darkfetch ${CMAKE_CURRENT_SOURCE_DIR}/src/darknet_image_fetch.cpp)
    add_dependencies(darkfetch smoke_gencpp ${smoke_EXPORTED_TARGETS})
    target_link_libraries(darkfetch ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS})

    add_executable(bboxtest ${CMAKE_CURRENT_SOURCE_DIR}/src/bbox_test_node.cpp)
    add_dependencies(bboxtest smoke_gencpp ${smoke_EXPORTED_TARGETS})
    target_link_libraries(bboxtest ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS})

    add_executable(srvtest ${CMAKE_CURRENT_SOURCE_DIR}/test/testSrv.cpp)
    add_dependencies(srvtest smoke_gencpp ${smoke_EXPORTED_TARGETS})
    target_link_libraries(srvtest ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS})

    if(Qt5_FOUND)
      set(CMAKE_AUTOMOC ON)
      set(CMAKE_AUTOUIC ON)
      set(CMAKE_AUTORCC ON)
      
      include_directories(${CMAKE_CURRENT_SOURCE_DIR}/appviz/include)
      file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} appviz/include/*.ui)
      file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} appviz/resources/*.qrc)
      file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} appviz/include/*.h)
      #qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
      #qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
      #qt5_wrap_cpp(QT_MOC_CPP ${QT_MOC})

      add_library(appviz_lib
                  ${QT_FORMS}
                  ${CMAKE_CURRENT_SOURCE_DIR}/appviz/src/mainwindow.cpp
                  ${CMAKE_CURRENT_SOURCE_DIR}/appviz/src/subnode.cpp
                 )
      target_link_libraries(appviz_lib Qt5::Core Qt5::Gui Qt5::Qml Qt5::Quick Qt5::Widgets
          ${QT_LIBRARIES} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

      #add_executable(appviz_node appviz/src/main.cpp ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_CPP})
      add_executable(appviz_node appviz/src/main.cpp ${QT_RESOURCES} ${QT_MOC})
      add_dependencies(appviz_node smoke_gencpp ${smoke_EXPORTED_TARGETS})
      target_link_libraries(appviz_node appviz_lib)
    endif()
  endif()

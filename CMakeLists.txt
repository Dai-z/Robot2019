cmake_minimum_required(VERSION 2.8.3)
project(imb)

add_compile_options(-std=c++11 -g)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation roscpp)

add_message_files(FILES ParticleInfo.msg AMCLInfo.msg AstarInfo.msg
                  MarkInfo.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES dviz
  # CATKIN_DEPENDS other_catkin_pkg
  # DEPENDS system_lib
)

find_package(
  Qt5
  COMPONENTS Core Widgets
  REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

include_directories(${catkin_INCLUDE_DIRS} include src)

add_executable(
  ${PROJECT_NAME}
  src/main.cpp
  src/mainwindow.cpp
  src/control_widget.cpp
  src/render.cpp
  src/model.cpp
  src/item_manager.cpp
  src/items/item.cpp
  src/items/field.cpp
  src/items/robot.cpp
  src/items/ball.cpp
  src/items/circle.cpp
  src/items/viewrange.cpp
  src/items/particles.cpp
  # ############################################################################
  include/mainwindow.hpp
  include/control_widget.hpp
  include/render.hpp
  include/model.hpp
  include/item_manager.hpp
  include/item.hpp
  include/field.hpp
  include/robot.hpp
  include/ball.hpp
  include/circle.hpp
  include/viewrange.hpp
  include/particles.hpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Core
                      Qt5::Widgets)

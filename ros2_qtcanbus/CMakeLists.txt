cmake_minimum_required(VERSION 3.8)
project(ros2_qtcanbus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 COMPONENTS Core SerialBus REQUIRED)
find_package(rclcpp REQUIRED)
find_package(libros2qt REQUIRED)

set(dependencies
    rclcpp
    std_msgs
    libros2qt
    ros2_qtcanbus_msgs
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


add_executable(qtcanbus_sender src/qtcanbus_sender.cpp src/qtcanbus_sender_node.cpp src/qtcanbus_sender_node.h)
set_target_properties(qtcanbus_sender PROPERTIES AUTOMOC TRUE)
target_include_directories(qtcanbus_sender PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(qtcanbus_sender PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(qtcanbus_sender Qt5::Core Qt5::SerialBus)

ament_target_dependencies(qtcanbus_sender ${dependencies})

install(TARGETS qtcanbus_sender
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

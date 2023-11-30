cmake_minimum_required(VERSION 3.8)
project(move_ekf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")


# find dependencies C++
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(GeographicLib REQUIRED)
ament_auto_find_build_dependencies()

# find dependencies python
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)


include_directories(include)

# C++ executables
ament_auto_add_executable(stamp_wamv src/stamp_wamv.cpp)
ament_auto_add_executable(waypoint_publisher src/waypoint_publisher.cpp)

install(TARGETS
    stamp_wamv
    waypoint_publisher
    DESTINATION lib/${PROJECT_NAME}
)


target_link_libraries(stamp_wamv ${GeographicLib_LIBRARIES})
# Add any other dependencies for waypoint_publisher if needed

# Python scripts installation
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/place_frame.py scripts/tracker.py
        DESTINATION lib/${PROJECT_NAME})

# Additional installation for non-Python files (if any)
install(DIRECTORY scripts/utm
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params
        DESTINATION share/${PROJECT_NAME})

ament_package()


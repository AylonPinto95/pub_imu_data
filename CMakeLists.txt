cmake_minimum_required(VERSION 3.8)
project(pub_imu_data)

set(IS_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/inertial-sense-sdk")
# message(${IS_SKD_DIR})


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(${IS_SDK_DIR} InertialSenseSDK)


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# Include directories
include_directories(
  ${LIBUSB_INCLUDE_DIRS}
  ${IS_SDK_DIR}/src
  ${IS_SDK_DIR}/src/libusb/libusb
)

link_directories(${IS_SDK_DIR})

add_library(cltoollib   
${IS_SDK_DIR}/cltool/src/cltool_main.cpp
${IS_SDK_DIR}/cltool/src/cltool.cpp)

# Add the executable
add_executable(imu_pub 
  src/publisher_member_function.cpp 
)
ament_target_dependencies(imu_pub rclcpp sensor_msgs)

# Link libusb
target_link_libraries(imu_pub cltoollib InertialSenseSDK Threads::Threads)



install(TARGETS
  imu_pub
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

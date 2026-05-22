cmake_minimum_required(VERSION 3.10)
project(ltyo_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE RelWithDebInfo)          # 或 Debug
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")  # 保险再加一行

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(base_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
set(CMAKE_SKIP_RPATH TRUE)

find_library(UNITREE_MOTOR_LIB
  NAMES libUnitreeMotorSDK_Linux64.so
  PATHS ${PROJECT_SOURCE_DIR}/lib
  )
  
add_executable(ltyo src/ltyo.cpp src/step.cpp src/TrotGait.cpp)

target_include_directories(ltyo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ltyo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  ltyo
  "rclcpp"
  "base_interfaces"
  "sensor_msgs"
)
install(TARGETS ltyo
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

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

cmake_minimum_required(VERSION 3.10)
project(motor_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_library(UNITREE_MOTOR_LIB
  NAMES libUnitreeMotorSDK_Arm64.so
  PATHS ${PROJECT_SOURCE_DIR}/lib
  )

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(base_interfaces REQUIRED)

file(GLOB src_files
        ${CMAKE_CURRENT_SOURCE_DIR}/src/unitree.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pid.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Tools.cpp
)

add_executable(unitree ${src_files})

target_include_directories(unitree PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(unitree PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  unitree
  "rclcpp"
  "base_interfaces"
)

target_link_libraries(unitree ${UNITREE_MOTOR_LIB})

install(TARGETS unitree
  DESTINATION lib/${PROJECT_NAME})

install(FILES ${UNITREE_MOTOR_LIB}
    DESTINATION lib
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

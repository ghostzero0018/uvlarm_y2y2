cmake_minimum_required(VERSION 3.8)
project(grp_pibot22)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# Python scripts
install( PROGRAMS scripts/robot_camera DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS scripts/basic_move DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS scripts/robot_bumper DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS scripts/scan_echo DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS scripts/detect_ghost DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS scripts/basic_move_test DESTINATION lib/${PROJECT_NAME} )


# Install resource files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

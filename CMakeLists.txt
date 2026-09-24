cmake_minimum_required(VERSION 3.16)

# C is required alongside CXX even though no C source belongs to this package:
# rosidl generates C sources for the messages below and links them into their own
# libraries, which CMake refuses to do without a C language enabled.
project(mrs_ultraloc LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

# The two sensors this node fuses live in their own repositories, so both have to
# be built before this one. Their generated messages are consumed from the
# installed headers rather than vendored: the field layout of UwbRangeStamped in
# particular is whatever the driver in the tree says it is, and a copy here would
# silently drift.
find_package(uvdar_core REQUIRED)
find_package(uwb_driver REQUIRED)

# FusionTargetArrayStamped is this package's own output type, so unlike the two
# inputs it is generated here.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FusionTarget.msg"
  "msg/FusionTargetArrayStamped.msg"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

add_executable(uwb_uvdar_fusion_node
  src/main_uwb_uvdar_fusion_node.cpp
  src/uwb_uvdar_fusion_node.cpp
)

target_include_directories(uwb_uvdar_fusion_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# First argument is the generation target, not the executable name.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(uwb_uvdar_fusion_node
  ${cpp_typesupport_target}
  Eigen3::Eigen
)

# uvdar_core and uwb_driver export their generated message headers, so
# ament_target_dependencies is what puts the bearing and range headers on the
# include path and links their typesupport libraries.
ament_target_dependencies(uwb_uvdar_fusion_node
  rclcpp
  std_msgs
  builtin_interfaces
  uvdar_core
  uwb_driver
)

install(TARGETS uwb_uvdar_fusion_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# The two simulator nodes. PROGRAMS, not TARGETS: these are Python scripts with a
# shebang, and this keeps them executable and puts them beside the node they are
# started next to, which is what `ros2 run mrs_ultraloc <name>` and a launch file's
# `executable=` both resolve.
install(PROGRAMS
  scripts/sim_uwb_module.py
  scripts/sim_uvdar_target.py
  DESTINATION lib/${PROJECT_NAME}
)

# The trajectory module the two scripts import. ament_python_install_package() is used
# rather than a plain install(DIRECTORY) because it is the one that registers the
# package's Python directory with the environment hooks, so `import sim_ultraloc` works
# in a sourced shell and in a test process without anything editing sys.path by hand.
#
# The first argument is the *import* name, not this package's name - it is also the
# directory installed under the Python dir and the name setuptools puts in the egg-info.
# It has to match the directory on disk for the import to resolve, which is why the
# module is called sim_ultraloc rather than being nested under mrs_ultraloc.
ament_python_install_package(sim_ultraloc
  PACKAGE_DIR sim_ultraloc
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

# | ---------------------------------------------------------------------------
# | Tests
# | ---------------------------------------------------------------------------
#
# Plain pytest over the real node process rather than launch_testing: the tests
# then run under `colcon test` and, identically, under `pytest test/test_fusion.py`
# against a sourced overlay, which is what makes them usable for a quick check on
# the vehicle. The node is launched by the test itself with its own topics, ids and
# a private ROS domain, so it cannot collide with a real setup.

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  set(_fusion_test_timeout 240)

  ament_add_pytest_test(test_fusion
    test/test_fusion.py
    TIMEOUT ${_fusion_test_timeout}
    # The tests own their output; without this a failure summary is hard to read.
    NOCAPTURE
    # A per-test workspace would be pointless, but the working directory has to be
    # the package so the relative test/ paths resolve however pytest is invoked.
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()

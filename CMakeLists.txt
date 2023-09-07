cmake_minimum_required(VERSION 3.8)
project(fast_lio)
set(CMAKE_CXX_STANDARD 14)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif ()

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
    include(ProcessorCount)
    ProcessorCount(N)
    message("Processer number:  ${N}")
    if (N GREATER 4)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=3)
        message("core for MP: 3")
    elseif (N GREATER 3)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=2)
        message("core for MP: 2")
    else ()
        add_definitions(-DMP_PROC_NUM=1)
    endif ()
else ()
    add_definitions(-DMP_PROC_NUM=1)
endif ()


find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(fast_lio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
#
#rosidl_generate_interfaces(${PROJECT_NAME}
#        "msg/Pose6D.msg"
#        DEPENDENCIES builtin_interfaces
#        LIBRARY_NAME ${PROJECT_NAME}
#        )
if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()
include_directories(${EIGEN3_INCLUDE_DIRS} include)
link_directories(lib/${PROJECT_NAME})
add_library(IMU_Processing src/IMU_Processing.cpp include/use-ikfom.cpp)
add_executable(${PROJECT_NAME}_node src/Fast_Lio.cpp src/fast_lio_node.cpp src/Preprocess.cpp include/ikd-Tree/ikd_Tree.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs pcl_ros tf2 nav_msgs geometry_msgs builtin_interfaces  fast_lio livox_ros_driver2)
ament_target_dependencies(IMU_Processing rclcpp sensor_msgs pcl_ros tf2 nav_msgs geometry_msgs builtin_interfaces  fast_lio livox_ros_driver2)
target_link_libraries(${PROJECT_NAME}_node IMU_Processing)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS IMU_Processing DESTINATION lib)
ament_package()

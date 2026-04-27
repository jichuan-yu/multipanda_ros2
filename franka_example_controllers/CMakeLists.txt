cmake_minimum_required(VERSION 3.5)
project(franka_example_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(generate_parameter_library)
find_package(franka_msgs REQUIRED)
find_package(multi_mode_control_msgs REQUIRED)
find_package(multi_mode_controller REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(dual_arm_reactive_control REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fcl REQUIRED)
find_package(OsqpEigen REQUIRED)

add_library(
        ${PROJECT_NAME}
        SHARED
        src/utils/robot_kinematics.cpp
        src/utils/collision_env.cpp
        src/utils/hierarchical_qp.cpp
        src/utils/constraint_manager.cpp
        src/utils/trajectory_buffer.cpp
        src/utils/trajectory_interpolator.cpp
        src/comless/gravity_compensation_example_controller.cpp
        src/comless/joint_impedance_example_controller.cpp
        src/comless/cartesian_impedance_example_controller.cpp
        src/comless/move_to_start_example_controller.cpp
        src/comless/joint_position_example_controller.cpp
        src/comless/joint_velocity_example_controller.cpp
        src/comless/cartesian_velocity_example_controller.cpp
        src/comless/motion_generator.cpp
        src/comless/model_example_controller.cpp
        src/comless/multi_joint_impedance_example_controller.cpp
        src/comless/dual_joint_velocity_example_controller.cpp
        
        src/subscriber/cartesian_impedance_controller.cpp
        src/subscriber/joint_impedance_controller.cpp
        src/subscriber/multi_joint_impedance_controller.cpp
        src/subscriber/multi_cartesian_impedance_controller.cpp
        src/subscriber/dualarm_mprc_controller.cpp
        )
        

target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
        /usr/local/include
        /usr/local/include/osqp
        /usr/local/share/osqp/codegen_files/inc/public
        /usr/local/share/osqp/codegen_files/inc/private
)

ament_target_dependencies(
        ${PROJECT_NAME}
        controller_interface
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        franka_semantic_components
        multi_mode_control_msgs
        multi_mode_controller
        std_msgs
        visualization_msgs
        dual_arm_reactive_control
        yaml-cpp
        fcl
        OsqpEigen
)

# Link external libraries manually
target_link_libraries(${PROJECT_NAME}
        fcl
        yaml-cpp
        OsqpEigen::OsqpEigen
)

generate_parameter_library(franka_example_controllers_parameters src/comless/model_example_controller_parameters.yaml)

add_executable(gripper_action_bridge src/subscriber/gripper_action_bridge.cpp)
ament_target_dependencies(gripper_action_bridge rclcpp rclcpp_action franka_msgs std_msgs std_srvs)
install(TARGETS gripper_action_bridge DESTINATION lib/${PROJECT_NAME})

pluginlib_export_plugin_description_file(
       controller_interface franka_example_controllers.xml)

install(
        TARGETS
        ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)

if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_copyright REQUIRED)
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)

    set(CPP_DIRECTORIES src include)
    ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
    ament_copyright(src ${CPP_DIRECTORIES} package.xml)
    ament_cppcheck(${CPP_DIRECTORIES})
    ament_lint_cmake(CMakeLists.txt)
    ament_flake8()
    ament_pep257()
    ament_xmllint()

    if(CHECK_TIDY)
        find_package(ament_cmake_clang_tidy REQUIRED)
        set(ament_cmake_clang_tidy_CONFIG_FILE ../.clang-tidy)
        ament_clang_tidy(${CMAKE_BINARY_DIR})
    endif()
endif()
ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        controller_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        hardware_interface
        franka_hardware
        multi_mode_control_msgs
)
ament_package()

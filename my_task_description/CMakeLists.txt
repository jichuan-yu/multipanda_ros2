cmake_minimum_required(VERSION 3.8)
project(my_task_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY mujoco launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
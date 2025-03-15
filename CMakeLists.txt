cmake_minimum_required(VERSION 3.5)
project(my_robot)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install the URDF and RViz files
install(DIRECTORY launch urdf rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

# Export the package dependencies
ament_package()

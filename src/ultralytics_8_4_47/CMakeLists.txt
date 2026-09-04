cmake_minimum_required(VERSION 3.0.2)
project(ultralytics_8_4_47)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  opencv.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Keep locally trained weights available when the package is installed.
install(DIRECTORY runs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)

# The launch file passes this directory to the Python3 node so a deployment
# without a pip-installed Ultralytics package can use the workspace copy.
install(DIRECTORY ultralytics
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

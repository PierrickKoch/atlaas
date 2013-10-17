test -z "${ROS_PACKAGE_PATH}" && source /opt/ros/groovy/setup.bash 
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`
rm -rf build; mkdir build; cd build; cmake ..; make -j8

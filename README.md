This is the personal project of Julian Denzel and Paul Pienkny.

Previous GIT Message:

# catkin_ws_denzelbeauty

	catkin config --profile odroid -b odroid-build -d odroid-devel -i odroid-install
	catkin config --profile odroid --cmake-args  -DCMAKE_TOOLCHAIN_FILE=`pwd`/src/Toolchain-arm-linux-gnueabihf.cmake
	catkin config --profile odroid --extend /opt/odroid-x2/sdk/opt/ros/indigo/
	catkin config --profile odroid --install

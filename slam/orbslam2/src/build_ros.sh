echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM2
if test -d "build"; then
	rm -rf build
fi
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j$(nproc)

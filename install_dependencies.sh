#Install OpenCV
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 4.0.0
cd ..

git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout 4.0.0
cd ..

cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D BUILD_EXAMPLES=OFF -D BUILD_opencv_apps=OFF -D BUILD_DOCS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc)
sudo make install
sudo sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig

cd ../../

#Install Eigen
sudo apt-get install libeigen3-dev

#Create a workspace
source /opt/ros/melodic/setup.bash
mkdir -p ~/allstar_ws/src
cd ~/allstar_ws/
catkin_make
source devel/setup.bash

#Install ROS Dependencies
sudo apt install libdxflib-dev
sudo apt install ros-melodic-map-server
sudo apt install ros-melodic-stage-ros

#Install all dependencies
cd ~/allstar_ws/src
git clone --branch allstar git@github.com:SamPusegaonkar/tuw_geometry.git 
git clone --branch allstar git@github.com:SamPusegaonkar/tuw_msgs.git 
git clone --branch allstar git@github.com:SamPusegaonkar/tuw_multi_robot.git 
git clone https://github.com/ShonBC/allstar

#Build the entire workspace!
cd ~/allstar_ws/
catkin_make


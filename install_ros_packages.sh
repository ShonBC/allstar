#Instlal turtlebot
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
  ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
  ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
  ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
  ros-melodic-rosserial-server ros-melodic-rosserial-client \
  ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
  ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
  ros-melodic-compressed-image-transport ros-melodic-rqt* \
  ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers


sudo apt-get install ros-melodic-dynamixel-sdk
sudo apt-get install ros-melodic-turtlebot3-msgs
sudo apt-get install ros-melodic-turtlebot3

cd

echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc

source ~/.bashrc

#Install all other ROS Packages
cd ~/allstar_ws/src
git clone --branch allstar git@github.com:SamPusegaonkar/tuw_geometry.git 
git clone --branch allstar git@github.com:SamPusegaonkar/tuw_msgs.git 
git clone --branch allstar git@github.com:SamPusegaonkar/tuw_multi_robot.git 
source ~/.bashrc
#Build the entire workspace!
cd ~/allstar_ws/
catkin_make

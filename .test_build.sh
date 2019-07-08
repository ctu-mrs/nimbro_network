sudo apt-get update -y
sudo apt-get upgrade -y

UAV_MODULES_PATH=`pwd`
ROS_WORKSPACE=~/mrs_workspace

git submodule update --init --recursive

rm -rf $ROS_WORKSPACE
mkdir -p $ROS_WORKSPACE/src

ln -s $UAV_MODULES_PATH $ROS_WORKSPACE/src/uav_modules

cd $ROS_WORKSPACE

catkin init
catkin config --extend /opt/ros/melodic

catkin config --profile default --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin config --profile release --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin profile set default

catkin build

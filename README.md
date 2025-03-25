# ros2_turtlesim_adapted
Adaption of the Ros2 humble tutlesim demo.  
Intended for the first step of the autonomous racing project, but other test changes might come.  

# Setup the environment (in Linux)
  -> ros2 has to be installed  
  
## Create environment and setup repository
cd ~  
mkdir ros2_autonomous_racing  
cd ros2_autonomous_racing  
mkdir src  
cd src  
mkdir turtlesim  
cd turtlesim  
git init  
git remote add origin git@github.com:FSCode8/ROS2_turtlesim_adapted.git  
git pull  
git checkout -b main origin/main  

## Build the package 
cd ~  
source /opt/ros/humble/setup.bash  
cd ros2_autonomous_racing/src/turtlesim  
colcon build --packages-select turtlesim  

## Run the a node of the package (has to be done in a new terminal)
source /opt/ros/humble/setup.bash 
cd ros2_autonomous_racing
source install/local_setup.bash  
ros2 run turtlesim turtlesim_node  

## Run the triangle paint function (other terminal with turtlesim_node has to be running)




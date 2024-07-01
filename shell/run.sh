cd ..
cd ..
cd ..
pwd

# catkin_make clean
catkin_make
source devel/setup.bash

roslaunch point_cloud_player point_cloud_player_node.launch
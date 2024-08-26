export COLCON_WS=~/workspaces/ur_gz
mkdir -p $COLCON_WS/src

cd $COLCON_WS
git clone -b ros2 https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation.git src/ur_simulation_gz
rosdep update && rosdep install --ignore-src --from-paths src -y
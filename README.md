# real-life-minesweeper

## environment launch

cd /cs-share/pradalier/CoppeliaSim
./coppeliaSim.sh
(open scene at : /cs-share/pradalier/ros2_ws/src/scenes/treasure_search.ttt)

ros2 run rviz2 rviz2
(topics /vrep/vision/image /vrep/kision/image and maybe )

ros2 run plotjuggler plotjuggler
(select vrep/metalDetector topic)

## setup the robot :

ros2 run joy joy_node
ros2 launch vrep_vsv_driver vsv_geom_mux.launch.py

ros2 launch vrep_vsv_driver vsv_pc_min.launch.py
# real-life-minesweeper

## environment launch

cd /cs-share/pradalier/CoppeliaSim && ./coppeliaSim.sh

(open scene at : /cs-share/pradalier/ros2_ws/src/scenes/treasure_search.ttt)

ros2 run rviz2 rviz2

(topics /vrep/vision/image /vrep/kision/image and type TF for axes)

ros2 run plotjuggler plotjuggler

(select vrep/metalDetector topic)

## setup the robot :

ros2 run joy joy_node

source install/setup.bash && ros2 launch vrep_vsv_driver vsv_geom_mux.launch.py

source install/setup.bash && ros2 launch vrep_vsv_driver vsv_pc_min.launch.py

## objective 1

source install/setup.bash && ros2 run minesweeper metalDetector

## objective 2 and 3

### creer un dataset

source install/setup.bash && ros2 launch shore_follower_observe record.launch.py

source install/setup.bash && ros2 launch shore_follower_regression record.launch.py

### entrainer un model (ex objective 2):

ssh -L 6006:localhost:6006 fjourda@balrog

cd /home/GTL/fjourda/ml4r/real-life-minesweeper

source /opt/venv/tf-gpu-ros2/bin/activate

colcon build --packages-select tensorflow_models_base

source install/setup.bash 

cd  tensorflow_models_base/tensorflow_models_base/

../scripts/train_regresion.sh 

(hyperparameter tuning in this last script)

vizualize curves with :

tensorboard --logdir /home/GTL/fjourda/ml4r/ws6_data_reg/output2

### tester :  

don't forget to load the proper env for this part (build cv_bridge and launch the node):

source /cs-share/pradalier/venv/tf-cpu-ros2/bin/activate

source install/setup.bash && ros2 launch minesweeper start.launch.py

source install/setup.bash && ros2 launch minesweeper start_reg.launch.py
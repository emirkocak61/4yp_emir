# 4yp_emir

This project involves motion and task planning solutions for a loco-manipulation problem for quadrupeds with arms. Currently, only simuylated for manipulating a needle valve. Other tests are coming soon. 

## Usage

In order to simulate the a full manipulation sequence:
* Run `roslaunch anytree_simulation anytree_test_environment.launch`. (This launches the simulation environment)
* Run the sim_ctrl executable under you unitree_ros/z1_controller package.
* Run `roslaunch anytree_simulation test.launch` (This launched the necessary action servers and controllers)
* `rosrun anytree_bt manioulate` (Runs the behaviour tree for manipulation)
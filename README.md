# rover_ros_ws

ROS packages for autonomous trajectory planning and tracking for IG32 Mecanum Ground Rover.

## Setup
Built and tested on Intel NUC with Ubuntu 20.04 and ROS Noetic.

1. Install dependencies: `pip install numpy scipy`
2. Install pytorch: https://pytorch.org/get-started/locally/ (select Stable, Linux, Pip, Python, CPU)
3. Setup ROS with Arduino (https://maker.pro/arduino/tutorial/how-to-use-arduino-with-robot-operating-system-ros)
4. Flash Arduino code: <repo link here>
5. Install catkin tools: https://catkin-tools.readthedocs.io/en/latest/installing.html
6. `git clone https://github.com/adamdai/rover_ros_ws.git`
7. `cd rover_ros_ws`
8. `catkin build`
9. `source devel/setup.bash`


## Packages
### controller
Nodes:
 - `traj_tracker.py`: Listens for published nominal trajectories and handles state estimation and low-level control for tracking.
### planner
Nodes:
 - `simple_planner.py`: Publishes a single nominal trajectory parameterized by desired linear and angular velocity (can be provided as command-line arguments, i.e., `rosrun planner simple_planner.py w_des v_des`).
 - `sequence_planner.py`: Publishes a pre-defined sequence of nominal trajectories 
 - `nn_planner.py`: Uses a pre-trained neural network to generate trajectories which avoid obstacles and reach a specified goal region.
 - `reachability_planner.py`: Uses the neural network planner with an added reachability safety layer which checks the safety of planned trajectories and replans if unsafe.
### sensing
Nodes:
 - `mocap.py`: Interface with vrpn mocap topic to provide mocap measurements.
### params
 - `params.py`: Edit global parameters.
### third_party
Contains all third party ROS packages used: `vrpn_client_ros`, `cv_camera`, `apriltag`,`apriltag_ros` 

## Usage

1. Start vrpn client: `roslaunch vrpn_client_ros sample.launch`
2. Start rosserial: `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0`
3. Start sensing nodes:
    - `rosrun sensing mocap.py`
    - `rosrun cv_camera cv_camera_node`
    - `roslaunch apriltag_ros continuous_detection.launch`
4. Start planning/control nodes:
    - `rosrun controller traj_tracker.py`
    - `rosrun planner simple_planner.py` or `rosrun planner nn_planner.py`

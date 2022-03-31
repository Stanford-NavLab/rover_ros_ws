# rover_ros_ws

ROS packages for autonomous trajectory planning and tracking for IG32 Mecanum Ground Rover.

## Setup
Built and tested on Intel NUC with Ubuntu 20.04 and ROS Noetic.

1. Install dependencies: `pip install numpy scipy`
2. Install pytorch: https://pytorch.org/get-started/locally/ (select Stable, Linux, Pip, Python, CPU)
3. Setup ROS with Arduino (https://maker.pro/arduino/tutorial/how-to-use-arduino-with-robot-operating-system-ros)
4. Install catkin tools: https://catkin-tools.readthedocs.io/en/latest/installing.html
5. `git clone https://github.com/adamdai/rover_ros_ws.git`
6. `cd rover_ros_ws`
7. `catkin build`
8. `source devel/setup.bash`


## Packages
### controller
Nodes:
 - `traj_tracker.py`
### planner
Nodes:
 - `simple_planner.py`
 - `sequence_planner.py`
 - `nn_planner.py`
 - `reachability_planner.py`
### sensing
Nodes:
 - `mocap.py`
### params
 - `params.py` 
### third_party
Contains all third party ROS packages used: `vrpn_client_ros`, `cv_camera`, `apriltag`,`apriltag_ros` 

## Usage

1. Start vrpn client: `roslaunch vrpn_client_ros sample.launch`
2. Start rosserial: `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0`
3. Start sensing nodes:
    - `rosrun sensing mocap.py`
4. Start planning/control nodes:
    - `rosrun controller traj_tracker.py`
    - `rosrun planner simple_planner.py` or `rosrun planner nn_planner.py`

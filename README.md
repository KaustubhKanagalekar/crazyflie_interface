
**This project was undertaken by Kaustubh Kanagalekar, Alexander La, and Yeiry Melendez (all part of SAS Lab)**
# Overview
This project implements a geofencing barrier for a Crazyflie drone using Control Barrier Functions (CBFs) in Python and ROS2. The goal was to apply safety corrections to the drone's nominal control inputs to keep it within a defined boundary.

## Implementation Details
Simulation:
The project began with a Python-based simulation where we experimented with heuristic values to understand the effects of a nominal controller and a CBF-based safety controller on a simplified drone model.



## Transition to Crazyflie:
After validating the CBF logic in simulation, we transitioned to a real Crazyflie drone. We implemented the CBF logic in a ROS2 node, which:

Integrated with a pre-existing LQR controller via the cf_interface to receive nominal control inputs. This LQR controller is different from the nominal controller defined above. 

Applied real-time safety corrections to keep the drone within a defined geofence.

Successfully achieved precise geofencing along a single axis (y or z) within a 2.5m x 2.5m enclosure.

## Results
Enabled real-time safety corrections using CBFs.

Improved operational stability and safety within the defined geofence.

Established a functional pipeline for future multi-axis geofencing


### Acknowledgements 
We would like to thank Sander Tonkens for his assistance throughout this project. In addition, we would like to thank SAS Lab for providing us with space and resources to conduct this project. 

The original README for the Crazyflie Interface Package is below- 
## Crazyflie Interface package
This package provides a basic interface to use low-level control (roll, pitch, yaw rate, thrust) to interface with the crazyflie 2.1.
It is meant as the base package upon which more elaborate controllers can be built.

The `template_controller.py` file provides the template to inherit for creating one owns controller. It can be imported as follows: `from crazyflie_interface_py.template_controller import TemplateController`.
- We provide a basic `lqr_controller.py` to showcase how to inherit from the `TemplateController`.
- The `template_controller.py` has the following base functionality (which can be overridden) implemented:
    - Keeps track of `self.state` (subscribed from the `cf_interface/state` topic)
    - Provides a wrapper around `__call__(self, state)`; `publish_control.py` which converts the control commands to ROS2 message and publishes to the `cf_interface/control` topic.
    - Sets the control rate & associated timer.


The base functionality (`ros2 launch crazyflie_interface launch.py`) does the following:
- Command line arguments: backend (currently: `sim`, `cflib`), uri (the robot id, for `cflib` backend)
- Launches the `crazyflie launch.py` launch file with the given backend and uri. This initiates:
    - Motion capture tracking (if `cflib` backend)
    - Connection to the crazyflie (if `cflib` backend)
    - Basic simulation engine (if `sim` backend)
- Launches the `cf_interface.py` node which:
    - Subscribes to the relevant crazyflie state topic, converts to standard form (see below) and publishes it to `cf_interface/state` topic.
    - Subscribes to the `cf_interface/control` topic, converts to crazyflie format and publishes the low-level control commands to the crazyflie
    - Provides the `cf_interface/command` service for high-level interface to crazyflie: `takeoff`, `land`
        - Example call: `ros2 service call /cf_interface/command crazyflie_interface/srv/Command "{command: 'takeoff'}"`
    

### Standard form
State message is as follows: [x, y, z, vx, vy, vz, q_x, q_y, q_z, q_w, wx, wy, wz]
- Linear velocities are in world frame
- Angular velocities are in body frame

Control message is as follows: [roll, pitch, yaw_rate, thrust]
- Roll, pitch are in radians
- Yaw rate is in radians per second
- Thrust is in Newtons per kilogram (same as gravity)

### Getting started

### Dependencies
- [UCSD SASLab's crazyswarm2 package](https://github.com/UCSD-SASLab/crazyswarm2).
- Tested on ROS2 Humble only.

### Installation
- `colcon build --packages-select crazyflie_interface` from `ros2_ws` directory.
- `source install/setup.bash` from `ros2_ws` directory.

### Running base functionality (sim)
1. `ros2 launch crazyflie_interface base_controller_launch.py backend:=sim` for simulation backend.
2. `ros2 service call /cf_interface/command crazyflie_interface/srv/Command "{command: 'takeoff'}"` to takeoff.
3. It will start flying to random position targets. (Note: with the new code, it will not travel to random targets, but rather to a single goal defined in the code) 
3. `ros2 service call /cf_interface/command crazyflie_interface/srv/Command "{command: 'land'}"` to land.


### Running base functionality (hardware)
1. Identify drone URI (# under drone)
2. `ros2 launch crazyflie_interface base_controller_launch.py backend:=cflib uri:=uri` for hardware backend.
3. `ros2 service call /cf_interface/command crazyflie_interface/srv/Command "{command: 'takeoff'}"` to takeoff.
4. It will start flying to random targets (Note: with the new code, it will not travel to random targets, but rather to a single goal defined in the code)
5. `ros2 service call /cf_interface/command crazyflie_interface/srv/Command "{command: 'land'}"` to land.

## TODOs
- [ ] Add GUI for command service.
- [ ] Add `gym_pybullet_drones` interface for simulation backend.
- [ ] Provide option for running rviz with `gym_pybullet_drones`.
- [ ] Add time to state message.
- [ ] Add calibration as part of command service in `cf_interface.py`.

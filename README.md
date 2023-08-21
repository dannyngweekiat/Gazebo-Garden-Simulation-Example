# Gazebo Garden Simulation Example

This is a simple example of using gazebo garden for ROS simulation.

## Pre-requisites

- Install Gazebo Garden from [https://gazebosim.org/docs/garden/install_ubuntu](https://gazebosim.org/docs/garden/install_ubuntu)
- Install Slam Toolbox [https://github.com/SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- Install Nav2 [https://navigation.ros.org/getting_started/index.html#installation](https://navigation.ros.org/getting_started/index.html#installation)

## Running the simulation

- Clone this repository into your workspace
- Build the workspace
- Source the workspace

## Running Basic Simulation

Run the following command to launch the simulation

```bash
ros2 launch gazebo_garden_simulation_example sim.launch.py 
```

Wait until the bridge is launch and in another terminal, run the following command to launch teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Running SLAM Simulation

Run the following command to launch the simulation

```bash
ros2 launch gazebo_garden_simulation_example sim.slam.launch.py
```

Wait until the bridge is launch and in another terminal start SLAM

```bash
ros2 launch gazebo_garden_simulation_example slam.launch.py
```

Wait until the map is built and in another terminal, run the following command to launch teleop to start mapping

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Running Navigation Simulation

Run the following command to launch the simulation

```bash
ros2 launch gazebo_garden_simulation_example sim.nav.launch.py
```

Wait until the bridge is launch and in another terminal start Navigation

```bash
ros2 launch gazebo_garden_simulation_example nav.launch.py
```

## Other Information

Modified version of [tugbot](https://app.gazebosim.org/MovAi/fuel/models/Tugbot) is used in the simulation. The modified version is available in the `models` folder.

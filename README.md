# mpc_controller

Follow these steps to run the ROS package:

1. Clone the GitHub repository into the `src` folder of your ROS workspace.
2. Build the package by running the following command in your workspace: `catkin build`
3. Launch the simulation stack with the command:
   - `roslaunch gbplanner rmf sim.launch gazebo_gui en:=true world file:= <path_to_world_file>`
4. Execute the NMPC node:
   roslaunch mpc_controller mpc.node
5. Run the planner in RViz

Alternatively, use the new launch file for automated setup:

- `roslaunch mpc_controller mpc.launch gazebo_gui:=true world_file:=<path_to_world_file>`

> **Note:** The `pci_general_ros.node` has been replaced with `waypoints.node.py`, which publishes a single desired waypoint.
   

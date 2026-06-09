Generated files for Gazebo Sim

Copy structure into your ROS2 package:
  ~/delta_ws/src/delta_robot/models/descripe/model.sdf
  ~/delta_ws/src/delta_robot/models/descripe/model.config
  ~/delta_ws/src/delta_robot/models/descripe/meshes/   <-- copy this mesh folder from SolidWorks export
  ~/delta_ws/src/delta_robot/worlds/descripe_test.world

Important changes:
- Converted passive joints to SDF type="ball":
  11, 12, 111, 121, 21, 22, 211, 221, 31, 32, 311, 321
- Kept the model as an open tree for loading.
- Added DetachableJoint plugins to close these links to end AFTER Gazebo starts:
  121, 211, 221, 311, 321, 411111

Run:
  export GZ_SIM_RESOURCE_PATH=$HOME/delta_ws/src/delta_robot/models:$GZ_SIM_RESOURCE_PATH
  gz sim -v 4 ~/delta_ws/src/delta_robot/worlds/descripe_test.world

Then, in another terminal:
  bash ~/delta_ws/src/delta_robot/close_loops.sh

If close_loops.sh fails, list actual topics:
  gz topic -l | grep close_loop

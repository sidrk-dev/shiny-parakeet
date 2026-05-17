# robomaster_arm_description config

This directory is reserved for RViz and MoveIt configuration files.

The first production MoveIt configuration should be generated after the real
link dimensions, masses, joint limits, and collision geometry are known. The
current `robot_arm.urdf.xacro` is intentionally a simple 6-DOF placeholder so
the ros2_control hardware path can be developed before CAD is final.

# Emergency Rescue Robot Core

Private-ready source snapshot for the emergency rescue robot ROS 1 project.

## What is included

- `catkin_ws/src/actuator`: chassis, arm, gimbal, and cannon ROS packages, URDFs, and robot description assets.
- `catkin_ws/src/user_msgs` and `catkin_ws/src/user_params`: custom ROS messages and hardware parameters.
- `catkin_ws/src/algorithm/detection`: AprilTag localization, cylinder detection, and the YOLOv5 RKNN integration source.
- `catkin_ws/src/algorithm/llm/deepseek`: the ROS integration source for the on-device language model.
- `mission_ws/src/mission`: mission management, waypoint PID navigation, and tag-based localization.

## Build and runtime dependencies

This is a source-only snapshot of a ROS 1 catkin project. Install the ROS distribution and ROS packages declared by the package manifests, then build `catkin_ws` before building `mission_ws` as an overlay. The exact ROS distribution and board setup should be confirmed against the target robot before deployment.

The YOLO/RKNN/RGA and DeepSeek/RKLLM packages require their vendor SDK headers and runtime libraries to be installed separately. The arm inverse-kinematics package also references a compiled kinematics library that is not included. Sensor and navigation drivers not listed above are external workspace dependencies.

## Device-specific configuration

Some scripts and YAML files contain absolute paths under `/home/orangepi/` and `/home/jetson/`, plus robot-specific camera, arm, waypoint, and map parameters. Replace or provision those paths and verify all parameters on the target robot before running any motion-control node. Mission recovery code can issue a reverse-velocity command; operate only with the robot secured and a clear test area.

## Repository scope

Generated catkin outputs, Python bytecode, SDK binaries and headers, model weights, datasets, archives, executables, maps, competition documents, and the LLaMA-Factory training data are intentionally excluded. Third-party ROS packages should be installed or managed separately with their own licenses and notices.

No project-wide license is declared in this snapshot. Confirm ownership and license compatibility for each package before changing this repository to public visibility.

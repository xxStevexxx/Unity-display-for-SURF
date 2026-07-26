# Depth Camera Simulation

Independent Unity project for Piper robotic arm simulation, ROS 2 control, and depth-camera / RealSense coordinate experiments.

This project was exported from the no-temperature-UI version of the Piper Unity ROS simulation. It does not include the headset temperature warning overlay.

## Features

- Unity Piper robotic arm simulation.
- ROS 2 communication through Unity Robotics ROS TCP Connector.
- Joint command and feedback topics for simulated arm motion.
- MoveIt trajectory action bridge to Unity joint commands.
- RealSense / depth-camera coordinate simulation helpers.
- Object target pose helper scripts for camera-based manipulation workflows.
- No temperature warning UI scripts or scene components.

## Requirements

- Unity `6000.5.0f1` or a compatible Unity 6 version.
- ROS 2 Humble environment.
- Unity Robotics ROS TCP Connector.
- Docker, if using the provided ROS / Piper SDK helper scripts.
- A built ROS TCP Endpoint workspace when running ROS communication.

## Open The Project

1. Open Unity Hub.
2. Click `Add` or `Open`.
3. Select this folder:

   ```text
   D:\Documents\SURF\depth camera simulation
   ```

4. Open the scene:

   ```text
   Assets/Scenes/SampleScene.unity
   ```

5. Press Play.

## Main Scripts

```text
Assets/Scripts/PiperArmController.cs
Assets/Scripts/PiperUnityArticulationBackend.cs
Assets/Scripts/PiperRosTopicBridge.cs
Assets/Scripts/PiperControlSystem.cs
Assets/Scripts/RealSenseCoordinateSimulator.cs
```

## ROS Topics

Important topics used by the Unity side include:

```text
/enable_cmd
/joint_ctrl_cmd
/joint_states_feedback
/joint_states
/arm_status
/arm_controller/follow_joint_trajectory
/gripper_controller/follow_joint_trajectory
```

## Useful Tools

```text
tools/piper_unity_menu.sh
tools/piper_moveit_to_unity_bridge.py
tools/piper_moveit_pose_planner.py
tools/auto_plan_detected_object_pose.py
tools/plan_detected_object_pose.sh
```

## Notes

This folder is a standalone Unity project directory. It is not a Git worktree and does not depend on the original repository checkout.

Unity-generated folders such as `Library/`, `Temp/`, `Logs/`, and `UserSettings/` are intentionally not included. Unity will recreate them when the project is opened.

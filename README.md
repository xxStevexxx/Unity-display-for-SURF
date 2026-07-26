# Piper Unity ROS Simulation

Unity project for simulating and controlling a Piper robotic arm through ROS 2.

This repository contains two project variants:

| Branch | Purpose |
| --- | --- |
| `with-temperature-display` | Full Unity project with the headset temperature warning overlay. |
| `without-temperature-display` | Same project without the temperature warning overlay. |
| `main` | Repository landing page. |

## Current Branch

This branch is `without-temperature-display`.

It keeps the Piper Unity simulation and ROS control workflow, but removes the temperature warning overlay scripts and scene components. Use this branch when you want a cleaner base simulation project.

## Features

- Unity Piper robotic arm simulation.
- ROS 2 communication through Unity Robotics ROS TCP Connector.
- Joint control and feedback topics for simulated arm motion.
- MoveIt trajectory action bridge to Unity joint commands.
- Optional real-arm helper scripts for Piper SDK / Docker workflows.
- RealSense coordinate simulation and object target helper scripts.
- No headset temperature warning UI in this branch.

## Requirements

- Unity `6000.5.0f1` or a compatible Unity 6 version.
- ROS 2 Humble environment.
- Unity Robotics ROS TCP Connector.
- Docker, if using the provided ROS / Piper SDK helper scripts.
- A built ROS TCP Endpoint workspace when running ROS communication.

## Open The Unity Project

1. Clone this branch:

   ```bash
   git clone -b without-temperature-display https://github.com/xxStevexxx/piper-unity-ros-sim.git
   ```

2. Open the cloned folder in Unity Hub.
3. Open `Assets/Scenes/SampleScene.unity`.
4. Press Play.

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

## Useful Scripts

```text
Assets/Scripts/PiperArmController.cs
Assets/Scripts/PiperUnityArticulationBackend.cs
Assets/Scripts/PiperRosTopicBridge.cs
Assets/Scripts/PiperControlSystem.cs
Assets/Scripts/RealSenseCoordinateSimulator.cs

tools/piper_unity_menu.sh
tools/piper_moveit_to_unity_bridge.py
tools/piper_moveit_pose_planner.py
tools/auto_plan_detected_object_pose.py
tools/plan_detected_object_pose.sh
```

## Version Difference

This branch does not include:

```text
Assets/Scripts/SafetyTemperatureZone.cs
Assets/Scripts/HeadsetSafetyWarningOverlay.cs
Assets/Scripts/SafetyWarningDemoBootstrapper.cs
```

If you need the safety temperature visualization, switch to:

```bash
git switch with-temperature-display
```

or choose `with-temperature-display` from the GitHub branch selector.

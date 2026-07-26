# Piper Unity ROS Simulation

Unity project for simulating and controlling a Piper robotic arm through ROS 2.

This repository contains two project variants:

| Branch | Purpose |
| --- | --- |
| `with-temperature-display` | Full Unity project with the headset temperature warning overlay. |
| `without-temperature-display` | Same project without the temperature warning overlay. |
| `main` | Repository landing page. |

## Current Branch

This branch is `with-temperature-display`.

It includes the Piper Unity simulation, ROS 2 topic bridge, MoveIt trajectory bridge tools, RealSense coordinate helper scripts, and a headset-facing temperature warning UI.

## Features

- Unity Piper robotic arm simulation.
- ROS 2 communication through Unity Robotics ROS TCP Connector.
- Joint control and feedback topics for simulated arm motion.
- MoveIt trajectory action bridge to Unity joint commands.
- Optional real-arm helper scripts for Piper SDK / Docker workflows.
- RealSense coordinate simulation and object target helper scripts.
- Headset safety warning overlay for temperature risk visualization.

## Temperature Warning Overlay

The temperature UI draws colored frames around detected or simulated environment objects in the headset view.

Risk levels:

| Average temperature | UI state |
| --- | --- |
| `< 40 C` | Safe, green |
| `40-60 C` | Caution, yellow |
| `60-80 C` | Danger, red |
| `>= 80 C` | Critical, flashing red |

The legend is shown in the upper-left corner of the headset view.

Important: this overlay is a visual warning system. It does not physically stop the robot unless you connect the warning state to the robot control logic.

Main temperature scripts:

```text
Assets/Scripts/SafetyTemperatureZone.cs
Assets/Scripts/HeadsetSafetyWarningOverlay.cs
Assets/Scripts/SafetyWarningDemoBootstrapper.cs
```

## Requirements

- Unity `6000.5.0f1` or a compatible Unity 6 version.
- ROS 2 Humble environment.
- Unity Robotics ROS TCP Connector.
- Docker, if using the provided ROS / Piper SDK helper scripts.
- A built ROS TCP Endpoint workspace when running ROS communication.

## Open The Unity Project

1. Clone this branch:

   ```bash
   git clone -b with-temperature-display https://github.com/xxStevexxx/piper-unity-ros-sim.git
   ```

2. Open the cloned folder in Unity Hub.
3. Open `Assets/Scenes/SampleScene.unity`.
4. Press Play.

If no manual `SafetyTemperatureZone` components exist in the scene, the demo bootstrapper creates neutral gray demo objects around the robot arm and displays warning frames for them.

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

## Switching Versions

Use the GitHub branch selector or run:

```bash
git switch with-temperature-display
git switch without-temperature-display
```

Use `with-temperature-display` for demonstrations that need safety temperature visualization.
Use `without-temperature-display` for a cleaner base simulation project.

# Piper Cube Grasp Simulation

This Unity project is an independent copy of the Piper arm temperature-warning simulation, extended for physical cube grasping tests.

Project folder:

```text
D:\Documents\SURF\piper-cube-grasp-simulation
```

The original temperature-warning Unity project is not modified.

## Version

Current working version: 2026-09-17

Branch:

```text
codex/realsense-point-cloud-2026-09-17
```

Main update in this version:

- physical cube grasping with repeated pickup and release;
- independent camera orbit control with arrow keys;
- full joint-by-joint keyboard control for the Piper arm;
- two grabbable cubes in the scene;
- gripper-mounted camera with V-key switching and a pulled-back view of the full gripper;
- translucent temperature surface overlays and adjacent labels adapting to both views;
- XYZ axis visual models removed without removing the coordinate reference;
- independent real-time colored RealSense PointCloud2 display;
- explicitly uncalibrated preview offset to keep the cloud above the demo platform;
- ROS2 point-cloud publisher and WSL/USB setup instructions included.

The point cloud is display-only: no surface fusion, collision mesh, temperature measurement, or automatic robot control. Preview positions are not calibrated robot coordinates. See [RealSense setup and limitations](REALTIME_POINT_CLOUD.md). The publisher is included at `tools/realsense_scene_publisher.py`; its existing workspace copy is unchanged.

Validation: 39 checks passed for decoding, preview placement, and orthographic/perspective GPU drawing with URP. D435 depth/color capture and ROS point-cloud publication were verified separately; this does not establish calibrated real-world alignment or full end-to-end hardware acceptance.

## Scene Objects

Open this scene:

```text
Assets/Scenes/SampleScene.unity
```

The demo contains two cube targets:

| Object | Size | Position | Notes |
| --- | --- | --- | --- |
| `GraspTargetCube` | `0.045 m` | `(0.5, 0.0245, 0.1)` | Original small cube target |
| `SecondaryGraspTargetCube` | `0.055 m` | `(0.5, 0.0295, -0.1)` | Slightly larger cube mirrored across the X axis |

Both cubes have physical collision. The larger cube is still small enough for the current gripper setup.

## How To Run

1. Open the project folder in Unity Hub.
2. Open `Assets/Scenes/SampleScene.unity`.
3. Press Play.
4. Press `Space` to enable the arm, or press any movement key if auto-enable is active.
5. Use the keyboard controls below to move the arm and grasp a cube.

## Keyboard Controls

Arm state:

```text
Space       Enable arm
Esc         Disable arm
0 / Num0 / Home
            Return arm to home pose
```

Joint control:

```text
A / D       Joint 1, base horizontal rotation
W / S       Joint 2, shoulder / middle arm motion
R / F       Joint 3, elbow / end section vertical motion
T / G       Joint 4, wrist roll
Y / H       Joint 5, wrist pitch
Q / E       Joint 6, end-effector rotation
O / P       Open / close gripper
```

Camera view:

```text
Left / Right Arrow     Rotate camera view horizontally
Up / Down Arrow        Rotate camera view vertically
V                      Switch overview / gripper view
```

The camera controls only rotate the viewing angle. They do not change the robot arm posture.

## Grasp Behavior

The grasp is physics-aware rather than a simple "close gripper equals grab" shortcut.

Current behavior:

- the cube keeps a collider and Rigidbody while it is on the platform;
- the cube can be pushed by the gripper before it is fully grasped;
- the object is considered grasped only when the gripper reaches a valid holding condition;
- opening the gripper releases the cube back to physics;
- after release, the cube can be picked up again;
- the visible temperature warning UI is not counted as part of the cube's physical size.

The gripper logic is implemented in:

```text
Assets/Scripts/PiperGripperGrabber.cs
Assets/Scripts/PiperGrabbableObject.cs
Assets/Scripts/PiperGraspTargetReset.cs
```

## Temperature Warning Overlay

This project keeps the temperature warning UI from the earlier version.

Risk levels:

| Average temperature | UI state |
| --- | --- |
| `< 40 C` | Safe, green |
| `40-65 C` | Caution, yellow |
| `65-120 C` | Danger, red |
| `>= 120 C` | Critical, flashing red |

Main temperature scripts:

```text
Assets/Scripts/SafetyTemperatureZone.cs
Assets/Scripts/HeadsetSafetyWarningOverlay.cs
Assets/Scripts/SafetyWarningDemoBootstrapper.cs
```

## Main Scripts

```text
Assets/Scripts/PiperArmController.cs
Assets/Scripts/PiperKeyboardController.cs
Assets/Scripts/PiperGameCameraLayout.cs
Assets/Scripts/PiperCubeGraspDemoBootstrapper.cs
Assets/Scripts/PiperGripperGrabber.cs
Assets/Scripts/PiperGrabbableObject.cs
Assets/Scripts/PiperGraspTargetReset.cs
```

## Notes

This is still a Unity simulation, not a real robot control stack. The aim is to make the grasping interaction closer to a real physical setup: the cube has volume, collision, gravity, and repeatable pickup/release behavior.

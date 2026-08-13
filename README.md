# Piper Cube Grasp Simulation

This Unity project is a copy of the temperature-warning Piper arm simulation, extended with a cube grasping demo.

The original temperature warning project is not modified. This folder is the independent working copy:

```text
D:\Documents\SURF\piper-cube-grasp-simulation
```

## What This Version Adds

- A grabbable cube target in the scene.
- A Rigidbody and Collider setup for the cube.
- A gripper trigger near the Piper end effector.
- Contact-based grab/release logic:
  - the cube is only held after both gripper fingers are close enough to the object;
  - the gripper opening is checked against the cube width;
  - opening the gripper releases the cube back to physics.
- A larger aligned box platform so the released cube lands on the visible surface.
- A re-grab state reset so the cube can be grabbed again after release.
- The existing headset temperature warning overlay is still included.

## How To Test The Grasp

1. Open this folder in Unity Hub.
2. Open `Assets/Scenes/SampleScene.unity`.
3. Press Play.
4. Press `Space` to enable the arm.
5. Rotate the camera view with the arrow keys.
6. Move the end effector near the cube:
   - `I` / `K`: move forward and backward;
   - `J` / `L`: move left and right;
   - `PageUp` / `PageDown`: move up and down;
   - hold `Shift` for fine control.
7. Press and hold `O` to open the gripper.
8. Move the gripper around the cube.
9. Press and hold `P` to close the gripper. The cube will attach only when both gripper fingers are close enough to the cube and the gripper opening matches the cube width.
10. Press `O` again to open the gripper and release the cube.

Joint control is still available:

```text
Q/A = joint 1
W/S = joint 2
E/D = joint 3
R/F = joint 4
T/G = joint 5
Y/H = joint 6
0 or Home = reset arm
Esc = disable arm
```

## Main Grasp Scripts

```text
Assets/Scripts/PiperCubeGraspDemoBootstrapper.cs
Assets/Scripts/PiperGripperGrabber.cs
Assets/Scripts/PiperGrabbableObject.cs
```

`PiperCubeGraspDemoBootstrapper` runs when the scene starts. It finds the Piper arm, prepares the cube, and creates a grab trigger at the gripper. This keeps the scene easy to open without manual Inspector setup.

## Temperature Warning Overlay

This project still keeps the temperature warning UI from the previous version.

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

## Notes

The grasp is still a Unity simulation, but it now checks object size, gripper opening, and two-finger contact before attaching the cube. After release, the cube returns to Rigidbody physics and can be grabbed again.

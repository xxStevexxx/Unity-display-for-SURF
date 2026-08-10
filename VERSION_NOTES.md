# Version Notes

## 2026-08-10 - Physical Cube Grasp Update

Branch: `grasp-physics-2026-08-10`

This version is based on the independent Unity project:

```text
D:\Documents\SURF\piper-cube-grasp-simulation
```

### Included Changes

- Added a cube grasping demo for the Piper robotic arm.
- Kept the existing temperature warning UI and hazard-zone scripts.
- Added contact-based grasping:
  - the cube cannot be grabbed before the gripper reaches it;
  - both gripper fingers must be close enough to the cube;
  - the gripper opening is compared with the cube width.
- Added release physics:
  - opening the gripper releases the cube;
  - the cube falls as a Rigidbody object;
  - release velocity is damped so the cube does not fly away.
- Fixed the platform:
  - enlarged the platform;
  - aligned the visible platform mesh with the BoxCollider;
  - prevented the cube from sinking into the platform on release.
- Fixed repeated grasping:
  - after release, old candidate/grasp state is cleared;
  - the cube can be grabbed again after a short cooldown.
- Removed the extra upper-right view overlays from the demo view.
- Preserved the cube's object color during grasp/release.

### Main Scripts

```text
Assets/Scripts/PiperCubeGraspDemoBootstrapper.cs
Assets/Scripts/PiperGripperGrabber.cs
Assets/Scripts/PiperGrabbableObject.cs
Assets/Scripts/PiperGraspTargetReset.cs
```

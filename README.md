# Unity Display For SURF

This repository contains Unity project variants for Piper robotic arm simulation, ROS 2 control, and underwater temperature-warning visualization.

## Branches

| Branch | Description |
| --- | --- |
| `with-temperature-display` | Unity project with the headset temperature warning overlay. |
| `without-temperature-display` | Unity project without the temperature warning overlay. |
| `main` | This landing page. |

Use the GitHub branch selector to choose the version you want.

## Temperature Display Version

The `with-temperature-display` branch uses subsea-manipulator warning thresholds based on underwater equipment temperature-rating guidance:

| Temperature | UI level |
| --- | --- |
| `< 40 C` | Safe |
| `40-65 C` | Caution |
| `65-120 C` | Danger |
| `>= 120 C` | Critical |

The overlay is intended as a visual warning system for underwater robotic-arm operation near hydrothermal-vent-style hot solids or hot flow regions.

## Quick Clone

With temperature display:

```bash
git clone -b with-temperature-display https://github.com/xxStevexxx/Unity-display-for-SURF.git
```

Without temperature display:

```bash
git clone -b without-temperature-display https://github.com/xxStevexxx/Unity-display-for-SURF.git
```

Open the cloned project in Unity Hub, then open:

```text
Assets/Scenes/SampleScene.unity
```

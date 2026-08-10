# Safety Warning UI

This module adds a headset-facing warning overlay for the existing Unity scene.

What it does:

- Simulates average temperature for objects around the robot arm.
- Draws colored UI frames around the corresponding objects in the headset view.
- Keeps the object materials unchanged. The warning color is only on the UI frame.
- Shows the average temperature inside each frame.
- Shows the color legend in the upper-left of the headset view:
  - T < 40 C: Safe, green
  - 40-65 C: Caution, yellow
  - 65-120 C: Danger, red
  - T >= 120 C: Critical, flashing red

How to run:

1. Open `Assets/Scenes/SampleScene.unity`.
2. Press Play.
3. The `Safety Warning Runtime` object is created automatically.
4. If no `SafetyTemperatureZone` components exist in the scene, neutral gray demo objects are created around the robot arm and framed by the headset UI.

To mark your own object as a temperature zone:

1. Select the scene object.
2. Add the `SafetyTemperatureZone` component.
3. Set `Zone Name` and `Base Temperature C`.
4. Press Play. The overlay will frame that object instead of relying only on demo objects.

Important design choice:

The system deliberately does not change the object's material color. It projects each object's bounds into the headset camera and draws a colored UI rectangle over the view, matching the requirement that warnings appear as headset annotations rather than object recoloring.

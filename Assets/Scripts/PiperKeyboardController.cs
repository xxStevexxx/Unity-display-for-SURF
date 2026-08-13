using UnityEngine;
using UnityEngine.InputSystem;

public sealed class PiperKeyboardController : MonoBehaviour
{
    [SerializeField] private float speedDegreesPerSecond = 45f;
    [SerializeField] private float gripperSpeedMetersPerSecond = 0.025f;
    [SerializeField] private bool autoEnableOnFirstInput = true;

    private PiperArmController arm;

    private void Awake()
    {
        arm = GetComponent<PiperArmController>();
        if (arm == null)
            arm = gameObject.AddComponent<PiperArmController>();
    }

    private void Update()
    {
        var keyboard = Keyboard.current;
        if (keyboard == null)
            return;

        if (keyboard[Key.Space].wasPressedThisFrame)
            arm.EnableArm();
        if (keyboard[Key.Escape].wasPressedThisFrame)
            arm.DisableArm();
        if (keyboard[Key.Digit0].wasPressedThisFrame || keyboard[Key.Numpad0].wasPressedThisFrame || keyboard[Key.Home].wasPressedThisFrame)
        {
            if (autoEnableOnFirstInput && !arm.IsEnabled)
                arm.EnableArm();
            arm.Home();
        }

        float step = speedDegreesPerSecond * Time.deltaTime;
        bool hasMotionInput = HasMotionInput(keyboard);
        if (hasMotionInput && autoEnableOnFirstInput && !arm.IsEnabled)
            arm.EnableArm();

        ApplyJointInput(keyboard, Key.D, Key.A, 0, step, -1f);
        ApplyJointInput(keyboard, Key.W, Key.S, 1, step);
        ApplyJointInput(keyboard, Key.R, Key.F, 2, step, -1f);
        ApplyJointInput(keyboard, Key.T, Key.G, 3, step);
        ApplyJointInput(keyboard, Key.Y, Key.H, 4, step);
        ApplyJointInput(keyboard, Key.E, Key.Q, 5, step);

        float gripperInput = 0f;
        if (keyboard[Key.O].isPressed)
            gripperInput += 1f;
        if (keyboard[Key.P].isPressed)
            gripperInput -= 1f;

        if (!Mathf.Approximately(gripperInput, 0f))
        {
            double currentOpening = arm.LastCommand?.GripperMeters ?? arm.Feedback?.GripperMeters ?? 0.0;
            arm.SetGripperMeters(currentOpening + gripperInput * gripperSpeedMetersPerSecond * Time.deltaTime);
        }
    }

    private bool HasMotionInput(Keyboard keyboard)
    {
        return HasPairInput(keyboard, Key.D, Key.A) ||
            HasPairInput(keyboard, Key.W, Key.S) ||
            HasPairInput(keyboard, Key.R, Key.F) ||
            HasPairInput(keyboard, Key.T, Key.G) ||
            HasPairInput(keyboard, Key.Y, Key.H) ||
            HasPairInput(keyboard, Key.E, Key.Q) ||
            HasPairInput(keyboard, Key.O, Key.P);
    }

    private void ApplyJointInput(
        Keyboard keyboard,
        Key positiveKey,
        Key negativeKey,
        int jointIndexZeroBased,
        float stepDegrees,
        float directionMultiplier = 1f)
    {
        float input = ReadSignedInput(keyboard, positiveKey, negativeKey);
        if (Mathf.Approximately(input, 0f))
            return;

        arm.AddJointDeltaDegrees(jointIndexZeroBased, input * stepDegrees * directionMultiplier);
    }

    private static bool HasPairInput(Keyboard keyboard, Key positiveKey, Key negativeKey)
    {
        return keyboard[positiveKey].isPressed || keyboard[negativeKey].isPressed;
    }

    private static float ReadSignedInput(Keyboard keyboard, Key positiveKey, Key negativeKey)
    {
        float input = 0f;
        if (keyboard[positiveKey].isPressed)
            input += 1f;
        if (keyboard[negativeKey].isPressed)
            input -= 1f;

        return input;
    }
}

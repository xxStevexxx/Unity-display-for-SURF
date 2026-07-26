using UnityEngine;
using UnityEngine.InputSystem;

public sealed class PiperKeyboardController : MonoBehaviour
{
    [SerializeField] private float speedDegreesPerSecond = 45f;
    [SerializeField] private float cartesianSpeedMetersPerSecond = 0.12f;
    [SerializeField] private float fineControlScale = 0.25f;
    [SerializeField] private bool cameraRelativeCartesianInput = true;
    [SerializeField] private int cartesianSolverIterations = 1;
    [SerializeField] private float cartesianSolverGain = 0.45f;
    [SerializeField] private float cartesianMaxJointStepDegrees = 2f;
    [SerializeField] private float gripperSpeedMetersPerSecond = 0.025f;
    [SerializeField] private bool autoEnableOnFirstInput = true;

    private PiperArmController arm;
    private readonly Key[] positiveKeys = { Key.Q, Key.W, Key.E, Key.R, Key.T, Key.Y };
    private readonly Key[] negativeKeys = { Key.A, Key.S, Key.D, Key.F, Key.G, Key.H };

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

        Vector3 cartesianInput = ReadCartesianInput(keyboard);
        if (cartesianInput.sqrMagnitude > 0f)
        {
            float speedScale = IsFineControlPressed(keyboard) ? Mathf.Clamp01(fineControlScale) : 1f;
            Vector3 worldDelta = ResolveCartesianInput(cartesianInput.normalized) *
                (cartesianSpeedMetersPerSecond * speedScale * Time.deltaTime);
            arm.TryMoveEndEffector(
                worldDelta,
                cartesianSolverIterations,
                cartesianSolverGain,
                cartesianMaxJointStepDegrees);
        }

        for (int i = 0; i < positiveKeys.Length; i++)
        {
            float input = 0f;
            if (keyboard[positiveKeys[i]].isPressed)
                input += 1f;
            if (keyboard[negativeKeys[i]].isPressed)
                input -= 1f;

            if (Mathf.Approximately(input, 0f))
                continue;

            arm.AddJointDeltaDegrees(i, input * step);
        }

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
        for (int i = 0; i < positiveKeys.Length; i++)
        {
            if (keyboard[positiveKeys[i]].isPressed || keyboard[negativeKeys[i]].isPressed)
                return true;
        }

        return keyboard[Key.O].isPressed || keyboard[Key.P].isPressed || ReadCartesianInput(keyboard).sqrMagnitude > 0f;
    }

    private static bool IsFineControlPressed(Keyboard keyboard)
    {
        return keyboard[Key.LeftShift].isPressed || keyboard[Key.RightShift].isPressed;
    }

    private static Vector3 ReadCartesianInput(Keyboard keyboard)
    {
        Vector3 input = Vector3.zero;
        if (keyboard[Key.RightArrow].isPressed)
            input.x += 1f;
        if (keyboard[Key.LeftArrow].isPressed)
            input.x -= 1f;
        if (keyboard[Key.UpArrow].isPressed)
            input.z += 1f;
        if (keyboard[Key.DownArrow].isPressed)
            input.z -= 1f;
        if (keyboard[Key.PageUp].isPressed)
            input.y += 1f;
        if (keyboard[Key.PageDown].isPressed)
            input.y -= 1f;

        return input;
    }

    private Vector3 ResolveCartesianInput(Vector3 input)
    {
        if (!cameraRelativeCartesianInput || Camera.main == null)
            return input;

        Transform cameraTransform = Camera.main.transform;
        Vector3 right = Vector3.ProjectOnPlane(cameraTransform.right, Vector3.up).normalized;
        Vector3 forward = Vector3.ProjectOnPlane(cameraTransform.forward, Vector3.up).normalized;
        if (right.sqrMagnitude < 0.0001f || forward.sqrMagnitude < 0.0001f)
            return input;

        Vector3 world = right * input.x + Vector3.up * input.y + forward * input.z;
        return world.sqrMagnitude > 1f ? world.normalized : world;
    }
}

using System;
using UnityEngine;

public sealed class PiperArmController : MonoBehaviour
{
    [Header("State")]
    [SerializeField] private PiperArmRunState initialState = PiperArmRunState.Disabled;
    [SerializeField] private bool autoEnableOnPlay;

    [Header("Motion")]
    [SerializeField] private float defaultSpeedPercent = 30f;
    [SerializeField] private float gripperEffort = 1f;

    [Header("Unity articulation drive")]
    [SerializeField] private float jointStiffness = 12000f;
    [SerializeField] private float jointDamping = 900f;
    [SerializeField] private float jointForceLimit = 1000f;
    [SerializeField] private float disabledJointStiffness = 0f;
    [SerializeField] private float disabledJointDamping = 20f;
    [SerializeField] private float disabledJointForceLimit = 80f;
    [SerializeField] private float linearDamping = 5f;
    [SerializeField] private float angularDamping = 5f;
    [SerializeField] private float jointFriction = 2f;

    private readonly PiperUnityArticulationBackend unityBackend = new();
    private IPiperArmBackend backend;
    private PiperJointCommand targetCommand = new();

    public PiperArmRunState RunState => backend?.Status.runState ?? PiperArmRunState.Offline;
    public bool IsEnabled => backend?.Status.enabled ?? false;
    public PiperArmStatus Status => backend?.Status;
    public PiperJointCommand Feedback => backend?.Feedback;
    public PiperJointCommand LastCommand => backend?.LastCommand;
    public float DefaultSpeedPercent => Mathf.Clamp(defaultSpeedPercent, 1f, 100f);
    public float GripperEffort => Mathf.Clamp(gripperEffort, 0.5f, 3f);
    public float JointStiffness => jointStiffness;
    public float JointDamping => jointDamping;
    public float JointForceLimit => jointForceLimit;
    public float DisabledJointStiffness => disabledJointStiffness;
    public float DisabledJointDamping => disabledJointDamping;
    public float DisabledJointForceLimit => disabledJointForceLimit;
    public float LinearDamping => linearDamping;
    public float AngularDamping => angularDamping;
    public float JointFriction => jointFriction;
    public bool TryGetEndEffectorPose(out Vector3 position, out Quaternion rotation)
    {
        if (backend == unityBackend)
            return unityBackend.TryGetEndEffectorPose(out position, out rotation);

        position = default;
        rotation = default;
        return false;
    }

    private void Awake()
    {
        backend = unityBackend;
        targetCommand.EnsureArrays();
        targetCommand.SpeedPercent = DefaultSpeedPercent;
        targetCommand.GripperEffort = GripperEffort;
        backend.Initialize(this);
        if (backend.LastCommand != null)
            targetCommand = backend.LastCommand.Clone();

        if (initialState == PiperArmRunState.Enabled || autoEnableOnPlay)
            EnableArm();
        else
            DisableArm();
    }

    private void Update()
    {
        backend?.Tick(Time.deltaTime);
    }

    public void EnableArm()
    {
        backend?.Enable();
    }

    public void DisableArm()
    {
        backend?.Disable();
    }

    public bool SetEnable(bool enable)
    {
        if (enable)
            EnableArm();
        else
            DisableArm();

        return IsEnabled == enable;
    }

    public void SetSpeedPercent(float speedPercent)
    {
        targetCommand.SpeedPercent = speedPercent;
        backend?.SendJointCommand(targetCommand);
    }

    public void SetJointTargetsRad(double[] jointRadians, float speedPercent = -1f)
    {
        targetCommand.EnsureArrays();
        int count = Math.Min(6, jointRadians?.Length ?? 0);
        for (int i = 0; i < count; i++)
            targetCommand.positionRad[i] = jointRadians[i];

        if (speedPercent > 0f)
            targetCommand.SpeedPercent = speedPercent;

        targetCommand.GripperEffort = GripperEffort;
        backend?.SendJointCommand(targetCommand);
    }

    public void SetSingleJointRad(int jointIndexZeroBased, double radians, float speedPercent = -1f)
    {
        if (jointIndexZeroBased < 0 || jointIndexZeroBased >= 6)
            return;

        targetCommand.EnsureArrays();
        targetCommand.positionRad[jointIndexZeroBased] = radians;
        if (speedPercent > 0f)
            targetCommand.SpeedPercent = speedPercent;

        backend?.SendJointCommand(targetCommand);
    }

    public void AddJointDeltaDegrees(int jointIndexZeroBased, float deltaDegrees)
    {
        if (jointIndexZeroBased < 0 || jointIndexZeroBased >= 6)
            return;

        targetCommand.EnsureArrays();
        targetCommand.positionRad[jointIndexZeroBased] += deltaDegrees * Mathf.Deg2Rad;
        backend?.SendJointCommand(targetCommand);
    }

    public void SetGripperMeters(double openingMeters, double effort = -1.0)
    {
        targetCommand.EnsureArrays();
        targetCommand.GripperMeters = openingMeters;
        targetCommand.GripperEffort = effort > 0.0 ? effort : GripperEffort;
        backend?.SendJointCommand(targetCommand);
    }

    public void Home(float speedPercent = -1f)
    {
        targetCommand.EnsureArrays();
        for (int i = 0; i < 6; i++)
            targetCommand.positionRad[i] = 0.0;

        targetCommand.GripperMeters = 0.0;
        targetCommand.GripperEffort = GripperEffort;
        if (speedPercent > 0f)
            targetCommand.SpeedPercent = speedPercent;

        backend?.SendJointCommand(targetCommand);
    }

    public void SendEndPose(PiperEndPoseCommand command)
    {
        backend?.SendEndPoseCommand(command);
    }

    public bool TryMoveEndEffector(Vector3 worldDelta, int iterations = 8, float gain = 0.45f, float maxJointStepDegrees = 2f)
    {
        if (backend == unityBackend)
        {
            bool moved = unityBackend.TryMoveEndEffector(worldDelta, iterations, gain, maxJointStepDegrees);
            if (moved && unityBackend.LastCommand != null)
                targetCommand = unityBackend.LastCommand.Clone();

            return moved;
        }

        return false;
    }
}

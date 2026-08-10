using System;
using UnityEngine;

public enum PiperControlTarget
{
    UnitySimulationRosTcp = 0,
    RealSdkRosTcpMirror = 1
}

[DefaultExecutionOrder(-50)]
public sealed class PiperControlSystem : MonoBehaviour
{
    [SerializeField] private PiperControlTarget target = PiperControlTarget.UnitySimulationRosTcp;
    [SerializeField] private bool enableKeyboardInUnityMode = true;
    [SerializeField] private bool publishUnityFeedback = true;
    [SerializeField] private bool publishUnityArmStatus = true;
    [SerializeField] private bool publishUnityEndPoseFeedback = true;
    [SerializeField] private bool acceptRosJointCommands = true;
    [SerializeField] private bool acceptRosEnableCommands = true;

    private PiperArmController arm;
    private PiperRosTopicBridge rosBridge;
    private PiperKeyboardController keyboard;

    public PiperControlTarget Target => target;
    public PiperArmRunState RunState => arm != null ? arm.RunState : PiperArmRunState.Offline;
    public bool IsEnabled => arm != null && arm.IsEnabled;

    private void Awake()
    {
        EnsureComponents();
        ApplyTargetMode();
    }

    private void OnValidate()
    {
        if (!Application.isPlaying)
            return;

        EnsureComponents();
        ApplyTargetMode();
    }

    public void SetTarget(PiperControlTarget nextTarget)
    {
        target = nextTarget;
        EnsureComponents();
        ApplyTargetMode();
    }

    public void EnableArm()
    {
        arm?.EnableArm();
    }

    public void DisableArm()
    {
        arm?.DisableArm();
    }

    public void Home(float speedPercent = -1f)
    {
        arm?.Home(speedPercent);
    }

    public void SetSpeedPercent(float speedPercent)
    {
        arm?.SetSpeedPercent(speedPercent);
    }

    public void SetJointTargetsDegrees(double[] jointDegrees, float speedPercent = -1f)
    {
        if (jointDegrees == null)
            return;

        int count = Math.Min(PiperRosContract.JointCount, jointDegrees.Length);
        var radians = new double[count];
        for (int i = 0; i < count; i++)
            radians[i] = jointDegrees[i] * Mathf.Deg2Rad;

        arm?.SetJointTargetsRad(radians, speedPercent);
    }

    private void EnsureComponents()
    {
        arm = GetComponent<PiperArmController>();
        if (arm == null)
            arm = gameObject.AddComponent<PiperArmController>();

        rosBridge = GetComponent<PiperRosTopicBridge>();
        if (rosBridge == null)
            rosBridge = gameObject.AddComponent<PiperRosTopicBridge>();

        keyboard = GetComponent<PiperKeyboardController>();
    }

    private void ApplyTargetMode()
    {
        if (rosBridge != null)
        {
            bool unityAuthority = target == PiperControlTarget.UnitySimulationRosTcp;
            rosBridge.SetRouting(
                publishUnityFeedback && unityAuthority,
                publishUnityArmStatus && unityAuthority,
                publishUnityEndPoseFeedback && unityAuthority,
                acceptRosJointCommands && unityAuthority,
                acceptRosEnableCommands && unityAuthority);
            rosBridge.enabled = unityAuthority;
        }

        if (keyboard != null)
            keyboard.enabled = target == PiperControlTarget.UnitySimulationRosTcp && enableKeyboardInUnityMode;
    }
}

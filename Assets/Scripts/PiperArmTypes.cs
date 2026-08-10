using System;
using UnityEngine;

public enum PiperArmRunState
{
    Offline,
    Disabled,
    Enabling,
    Enabled,
    Disabling,
    Fault
}

public enum PiperControlMode
{
    EndPose = 0,
    Joint = 1
}

[Serializable]
public sealed class PiperJointCommand
{
    public string[] names = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper" };
    public double[] positionRad = new double[7];
    public double[] velocity = new double[7];
    public double[] effort = new double[7];
    public int mode1 = 0x01;
    public int mode2 = 0x01;

    public float SpeedPercent
    {
        get => velocity != null && velocity.Length >= 7 ? Mathf.Clamp((float)velocity[6], 1f, 100f) : 100f;
        set
        {
            EnsureArrays();
            velocity[6] = Mathf.Clamp(value, 1f, 100f);
        }
    }

    public double GripperMeters
    {
        get => positionRad != null && positionRad.Length >= 7 ? positionRad[6] : 0.0;
        set
        {
            EnsureArrays();
            positionRad[6] = Math.Max(0.0, Math.Min(0.08, value));
        }
    }

    public double GripperEffort
    {
        get => effort != null && effort.Length >= 7 ? effort[6] : 1.0;
        set
        {
            EnsureArrays();
            effort[6] = Math.Max(0.5, Math.Min(3.0, value));
        }
    }

    public void EnsureArrays()
    {
        if (positionRad == null || positionRad.Length != 7)
            positionRad = new double[7];
        if (velocity == null || velocity.Length != 7)
            velocity = new double[7];
        if (effort == null || effort.Length != 7)
            effort = new double[7];
        if (names == null || names.Length != 7)
            names = new[] { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper" };
    }

    public PiperJointCommand Clone()
    {
        EnsureArrays();
        var copy = new PiperJointCommand
        {
            names = (string[])names.Clone(),
            positionRad = (double[])positionRad.Clone(),
            velocity = (double[])velocity.Clone(),
            effort = (double[])effort.Clone(),
            mode1 = mode1,
            mode2 = mode2
        };
        return copy;
    }
}

[Serializable]
public sealed class PiperArmStatus
{
    public byte ctrlMode;
    public byte armStatus;
    public byte modeFeedback;
    public byte teachStatus;
    public byte motionStatus;
    public byte trajectoryNum;
    public long errCode;
    public bool[] jointAngleLimit = new bool[6];
    public bool[] jointCommunicationOk = new bool[6];
    public PiperArmRunState runState = PiperArmRunState.Disabled;
    public bool enabled;
}

[Serializable]
public sealed class PiperEndPoseCommand
{
    public double x;
    public double y;
    public double z;
    public double roll;
    public double pitch;
    public double yaw;
    public double gripper;
    public int mode1;
    public int mode2;
}

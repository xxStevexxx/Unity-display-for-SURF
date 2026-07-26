public static class PiperRosContract
{
    public const string JointFeedbackTopic = "/joint_states_feedback";
    public const string JointCommandTopic = "/joint_ctrl_cmd";
    public const string ArmStatusTopic = "/arm_status";
    public const string EndPoseFeedbackTopic = "/link6_pose";
    public const string EnableCommandTopic = "/enable_cmd";
    public const string EnableService = "/enable_srv";
    public const string EndPoseCommandTopic = "/pos_cmd";

    public const int JointCount = 6;
    public const int JointStateElementCount = 7;
    public const double RealArmJointRawUnitsPerRadian = 57324.840764;
    public const double RealArmGripperRawUnitsPerMeter = 1000000.0;
    public const double RealArmPosePositionRawUnitsPerMeter = 1000000.0;
    public const double RealArmPoseRotationRawUnitsPerRadian = 57295.779513;
    public const float MinSpeedPercent = 1f;
    public const float MaxSpeedPercent = 100f;
}

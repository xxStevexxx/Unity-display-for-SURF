using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Piper;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public sealed class PiperRosTopicBridge : MonoBehaviour
{
    [SerializeField] private bool connectOnStart = true;
    [SerializeField] private float publishHz = 50f;
    [SerializeField] private bool publishJointFeedback = true;
    [SerializeField] private bool publishArmStatus = true;
    [SerializeField] private bool publishEndPoseFeedback = true;
    [SerializeField] private bool acceptJointCommands = true;
    [SerializeField] private bool acceptEnableCommands = true;
    [SerializeField] private bool autoEnableOnJointCommand = true;
    [SerializeField] private string jointFeedbackTopic = PiperRosContract.JointFeedbackTopic;
    [SerializeField] private string jointCommandTopic = PiperRosContract.JointCommandTopic;
    [SerializeField] private string armStatusTopic = PiperRosContract.ArmStatusTopic;
    [SerializeField] private string endPoseFeedbackTopic = PiperRosContract.EndPoseFeedbackTopic;
    [SerializeField] private string endPoseFrameId = "base_link";
    [SerializeField] private Transform endPoseReferenceFrame;
    [SerializeField] private string enableCommandTopic = PiperRosContract.EnableCommandTopic;

    private PiperArmController arm;
    private ROSConnection ros;
    private float nextPublishTime;

    private void Awake()
    {
        arm = GetComponent<PiperArmController>();
        if (arm == null)
            arm = gameObject.AddComponent<PiperArmController>();
    }

    private void Start()
    {
        nextPublishTime = 0f;

        if (!connectOnStart)
            return;

        Connect();
    }

    public void SetRouting(bool publishFeedback, bool publishStatus, bool publishEndPose, bool subscribeJointCommands, bool subscribeEnableCommands)
    {
        publishJointFeedback = publishFeedback;
        publishArmStatus = publishStatus;
        publishEndPoseFeedback = publishEndPose;
        acceptJointCommands = subscribeJointCommands;
        acceptEnableCommands = subscribeEnableCommands;
    }

    public void Connect()
    {
        ros = ROSConnection.GetOrCreateInstance();

        var jointFeedbackState = ros.GetTopic(jointFeedbackTopic);
        var armStatusState = ros.GetTopic(armStatusTopic);
        var endPoseFeedbackState = ros.GetTopic(endPoseFeedbackTopic);

        if (publishJointFeedback && (jointFeedbackState == null || !jointFeedbackState.IsPublisher))
            ros.RegisterPublisher<JointStateMsg>(jointFeedbackTopic);
        if (publishArmStatus && (armStatusState == null || !armStatusState.IsPublisher))
            ros.RegisterPublisher<PiperStatusMsg>(armStatusTopic);
        if (publishEndPoseFeedback && (endPoseFeedbackState == null || !endPoseFeedbackState.IsPublisher))
            ros.RegisterPublisher<PoseStampedMsg>(endPoseFeedbackTopic);
        if (acceptJointCommands && !ros.HasSubscriber(jointCommandTopic))
            ros.Subscribe<JointStateMsg>(jointCommandTopic, OnJointCommand);
        if (acceptEnableCommands && !ros.HasSubscriber(enableCommandTopic))
            ros.Subscribe<BoolMsg>(enableCommandTopic, OnEnableCommand);
    }

    private void Update()
    {
        if (ros == null || arm == null || Time.time < nextPublishTime)
            return;

        nextPublishTime = Time.time + 1f / Mathf.Max(1f, publishHz);
        if (publishJointFeedback)
            PublishJointFeedback();
        if (publishArmStatus)
            PublishArmStatus();
        if (publishEndPoseFeedback)
            PublishEndPoseFeedback();
    }

    private void PublishJointFeedback()
    {
        var feedback = arm.Feedback;
        if (feedback == null)
            return;

        feedback.EnsureArrays();
        var msg = new JointStateMsg
        {
            header = MakeHeader("base_link"),
            name = (string[])feedback.names.Clone(),
            position = (double[])feedback.positionRad.Clone(),
            velocity = (double[])feedback.velocity.Clone(),
            effort = (double[])feedback.effort.Clone()
        };

        ros.Publish(jointFeedbackTopic, msg);
    }

    private void PublishArmStatus()
    {
        var status = arm.Status;
        if (status == null)
            return;

        bool[] limits = status.jointAngleLimit ?? new bool[6];
        bool[] comms = status.jointCommunicationOk ?? new bool[6];
        var msg = new PiperStatusMsg
        {
            ctrl_mode = status.ctrlMode,
            arm_status = status.armStatus,
            mode_feedback = status.modeFeedback,
            teach_status = status.teachStatus,
            motion_status = status.motionStatus,
            trajectory_num = status.trajectoryNum,
            err_code = status.errCode,
            joint_1_angle_limit = GetBool(limits, 0),
            joint_2_angle_limit = GetBool(limits, 1),
            joint_3_angle_limit = GetBool(limits, 2),
            joint_4_angle_limit = GetBool(limits, 3),
            joint_5_angle_limit = GetBool(limits, 4),
            joint_6_angle_limit = GetBool(limits, 5),
            communication_status_joint_1 = GetBool(comms, 0),
            communication_status_joint_2 = GetBool(comms, 1),
            communication_status_joint_3 = GetBool(comms, 2),
            communication_status_joint_4 = GetBool(comms, 3),
            communication_status_joint_5 = GetBool(comms, 4),
            communication_status_joint_6 = GetBool(comms, 5)
        };

        ros.Publish(armStatusTopic, msg);
    }

    private void PublishEndPoseFeedback()
    {
        if (!arm.TryGetEndEffectorPose(out Vector3 worldPosition, out Quaternion worldRotation))
            return;

        Transform referenceFrame = endPoseReferenceFrame != null ? endPoseReferenceFrame : arm.transform;
        Vector3 localPosition = referenceFrame.InverseTransformPoint(worldPosition);
        Quaternion localRotation = Quaternion.Inverse(referenceFrame.rotation) * worldRotation;
        PointMsg rosPosition = localPosition.To<FLU>();
        QuaternionMsg rosRotation = localRotation.To<FLU>();

        var msg = new PoseStampedMsg
        {
            header = MakeHeader(endPoseFrameId),
            pose = new PoseMsg(rosPosition, rosRotation)
        };

        ros.Publish(endPoseFeedbackTopic, msg);
    }

    private void OnJointCommand(JointStateMsg msg)
    {
        if (arm == null || msg == null)
            return;

        var command = arm.LastCommand != null ? arm.LastCommand.Clone() : new PiperJointCommand();
        command.EnsureArrays();

        for (int i = 0; i < msg.name.Length; i++)
        {
            int index = JointNameToIndex(msg.name[i]);
            if (index < 0 || index >= command.positionRad.Length)
                continue;

            if (msg.position != null && i < msg.position.Length)
                command.positionRad[index] = msg.position[i];
            if (msg.velocity != null && i < msg.velocity.Length)
                command.velocity[index] = msg.velocity[i];
            if (msg.effort != null && i < msg.effort.Length)
                command.effort[index] = msg.effort[i];
        }

        if (msg.velocity != null && msg.velocity.Length >= 7)
            command.SpeedPercent = (float)msg.velocity[6];

        if (autoEnableOnJointCommand && !arm.IsEnabled)
            arm.EnableArm();

        arm.SetJointTargetsRad(command.positionRad, command.SpeedPercent);
        arm.SetGripperMeters(command.GripperMeters, command.GripperEffort);
    }

    private void OnEnableCommand(BoolMsg msg)
    {
        if (arm != null && msg != null)
            arm.SetEnable(msg.data);
    }

    private static int JointNameToIndex(string jointName)
    {
        return jointName switch
        {
            "joint1" => 0,
            "joint2" => 1,
            "joint3" => 2,
            "joint4" => 3,
            "joint5" => 4,
            "joint6" => 5,
            "joint7" => 6,
            "gripper" => 6,
            _ => -1
        };
    }

    private static bool GetBool(bool[] values, int index)
    {
        return values != null && index >= 0 && index < values.Length && values[index];
    }

    private static HeaderMsg MakeHeader(string frameId)
    {
        double now = Time.timeAsDouble;
        int sec = Mathf.FloorToInt((float)now);
        uint nanosec = (uint)((now - sec) * 1000000000.0);
        var header = new HeaderMsg();
        var stamp = new TimeMsg();
#if ROS2
        stamp.sec = sec;
#else
        stamp.sec = (uint)sec;
#endif
        stamp.nanosec = nanosec;
        header.stamp = stamp;
        header.frame_id = frameId;
        return header;
    }
}

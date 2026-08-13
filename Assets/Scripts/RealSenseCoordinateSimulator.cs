using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public sealed class RealSenseCoordinateSimulator : MonoBehaviour
{
    [Header("Scene Frames")]
    [SerializeField] private Transform baseLinkFrame;
    [SerializeField] private Transform cameraOpticalFrame;
    [SerializeField] private Transform targetObject;
    [SerializeField] private Transform convertedTargetMarker;

    [Header("ROS Publishing")]
    [SerializeField] private bool publishToRos = false;
    [SerializeField] private string poseTopic = "/detected_object_pose";
    [SerializeField] private string frameId = "base_link";
    [SerializeField] private float publishHz = 10f;

    [Header("Debug")]
    [SerializeField] private bool logEverySecond = true;

    private ROSConnection ros;
    private bool posePublisherRegistered;
    private float nextPublishTime;
    private float nextPublishLogTime;
    private float nextLogTime;

    public Vector3 CameraXyzMeters { get; private set; }
    public Vector3 BaseLinkXyzMeters { get; private set; }

    public void SetTargetObject(Transform target)
    {
        targetObject = target;
    }

    private void Start()
    {
        if (!publishToRos)
            return;

        ros = ROSConnection.GetOrCreateInstance();
        var topicState = ros.GetTopic(poseTopic);
        if (topicState == null || !topicState.IsPublisher)
            ros.RegisterPublisher<PoseStampedMsg>(poseTopic);

        posePublisherRegistered = true;
        nextPublishTime = Time.time + 0.5f;
        Debug.Log($"Registered ROS publisher for {poseTopic}");
    }

    private void Update()
    {
        if (baseLinkFrame == null || cameraOpticalFrame == null || targetObject == null)
            return;

        CameraXyzMeters = WorldPointToRealSenseCamera(targetObject.position);
        BaseLinkXyzMeters = CameraPointToBaseLink(CameraXyzMeters);

        if (convertedTargetMarker != null)
            convertedTargetMarker.position = BaseLinkPointToWorld(BaseLinkXyzMeters);

        if (logEverySecond && Time.time >= nextLogTime)
        {
            nextLogTime = Time.time + 1f;
            Debug.Log(
                $"RealSense camera xyz = {Format(CameraXyzMeters)} m, " +
                $"base_link xyz = {Format(BaseLinkXyzMeters)} m");
        }

        if (publishToRos && posePublisherRegistered && ros != null && Time.time >= nextPublishTime)
        {
            nextPublishTime = Time.time + 1f / Mathf.Max(1f, publishHz);
            PublishBaseLinkPose(BaseLinkXyzMeters);
        }
    }

    private Vector3 WorldPointToRealSenseCamera(Vector3 worldPoint)
    {
        Vector3 cameraUnity = cameraOpticalFrame.InverseTransformPoint(worldPoint);
        return new Vector3(cameraUnity.x, -cameraUnity.y, cameraUnity.z);
    }

    private Vector3 CameraPointToBaseLink(Vector3 cameraPoint)
    {
        Vector3 cameraUnity = new(cameraPoint.x, -cameraPoint.y, cameraPoint.z);
        Vector3 worldPoint = cameraOpticalFrame.TransformPoint(cameraUnity);
        Vector3 baseUnity = baseLinkFrame.InverseTransformPoint(worldPoint);

        return new Vector3(baseUnity.z, -baseUnity.x, baseUnity.y);
    }

    private Vector3 BaseLinkPointToWorld(Vector3 baseLinkPoint)
    {
        Vector3 baseUnity = new(-baseLinkPoint.y, baseLinkPoint.z, baseLinkPoint.x);
        return baseLinkFrame.TransformPoint(baseUnity);
    }

    private void PublishBaseLinkPose(Vector3 baseLinkPoint)
    {
        var msg = new PoseStampedMsg
        {
            header = MakeHeader(frameId),
            pose = new PoseMsg
            {
                position = new PointMsg
                {
                    x = baseLinkPoint.x,
                    y = baseLinkPoint.y,
                    z = baseLinkPoint.z
                },
                orientation = new QuaternionMsg
                {
                    x = 0.0,
                    y = 0.0,
                    z = 0.0,
                    w = 1.0
                }
            }
        };

        ros.Publish(poseTopic, msg);

        if (logEverySecond && Time.time >= nextPublishLogTime)
        {
            nextPublishLogTime = Time.time + 1f;
            Debug.Log($"Published {poseTopic}: base_link xyz = {Format(baseLinkPoint)} m");
        }
    }

    private static HeaderMsg MakeHeader(string frame)
    {
        double now = Time.timeAsDouble;
        int sec = Mathf.FloorToInt((float)now);
        uint nanosec = (uint)((now - sec) * 1000000000.0);

        var stamp = new TimeMsg();
#if ROS2
        stamp.sec = sec;
#else
        stamp.sec = (uint)sec;
#endif
        stamp.nanosec = nanosec;

        return new HeaderMsg
        {
            stamp = stamp,
            frame_id = frame
        };
    }

    private void OnDrawGizmos()
    {
        if (baseLinkFrame != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(baseLinkFrame.position, 0.04f);
        }

        if (cameraOpticalFrame != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(cameraOpticalFrame.position, 0.04f);
        }

        if (targetObject != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(targetObject.position, 0.05f);
        }

        if (cameraOpticalFrame != null && targetObject != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(cameraOpticalFrame.position, targetObject.position);
        }

        if (baseLinkFrame != null && targetObject != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(baseLinkFrame.position, targetObject.position);
        }
    }

    private static string Format(Vector3 value)
    {
        return $"({value.x:F3}, {value.y:F3}, {value.z:F3})";
    }
}

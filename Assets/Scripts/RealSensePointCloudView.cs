using System;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.Rendering;

[DisallowMultipleComponent]
public sealed class RealSensePointCloudView : MonoBehaviour
{
    [Header("ROS (configure before Play)")]
    public string topic = "/realsense/obstacle_points";
    public string expectedFrameId = "base_link";
    public RealSensePointCloudDecoder.Coordinates coordinates = RealSensePointCloudDecoder.Coordinates.RosFlu;
    [Tooltip("Unity transform matching the message frame after axis conversion. Requires calibrated pose, unit scale.")]
    public Transform frameOrigin;
    [Header("Display only - no physics or robot commands")]
    [Tooltip("Preview translation only, NOT camera/robot calibration. Disable after calibrating Frame Origin and the publisher extrinsics.")]
    public bool usePreviewOffset = true;
    public Vector3 previewWorldOffset = new(1.1f, 0.85f, -0.35f);
    [Range(100, 100000)] public int maxPoints = 30000;
    [Range(0.001f, 0.05f)] public float pointDiameter = 0.008f;
    [Range(1, 30)] public float refreshHz = 10;
    [Min(0.1f)] public float staleSeconds = 2;

    public string Status { get; private set; } = "Waiting for Play";
    public int VisiblePoints { get; private set; }
    private readonly List<Vector3> positions = new();
    private readonly List<Color32> colors = new();
    private readonly List<Vector3> vertices = new();
    private readonly List<Color32> vertexColors = new();
    private readonly List<Vector2> uv = new();
    private readonly List<int> triangles = new();
    private readonly object gate = new();
    private PointCloud2Msg pending;
    private Mesh mesh;
    private Material material;
    private float nextRefresh;
    private float lastValidFrame = float.NegativeInfinity;
    private float nextWarning;
    private bool subscribed;
    private bool loggedFirstCloud;

    public Matrix4x4 DisplayMatrix
    {
        get
        {
            Transform origin = frameOrigin != null ? frameOrigin : transform;
            Vector3 offset = usePreviewOffset ? previewWorldOffset : Vector3.zero;
            return Matrix4x4.TRS(origin.position + offset, origin.rotation, Vector3.one);
        }
    }

    private void Start()
    {
        var shader = Resources.Load<Shader>("RealSensePointCloud");
        if (shader == null || !shader.isSupported)
        {
            Status = "Point cloud shader missing or unsupported";
            Debug.LogError(Status, this);
            enabled = false;
            return;
        }
        material = new Material(shader) { hideFlags = HideFlags.HideAndDontSave };
        mesh = new Mesh { name = "Live RealSense points", indexFormat = IndexFormat.UInt32 };
        mesh.MarkDynamic();
        if (string.IsNullOrWhiteSpace(topic) || string.IsNullOrWhiteSpace(expectedFrameId))
        {
            Status = "Set topic and expected frame ID before Play";
            enabled = false;
            return;
        }
        // Connector has only UnsubscribeAll. A weak callback avoids deleting other subscribers.
        var target = new WeakReference<RealSensePointCloudView>(this);
        ROSConnection.GetOrCreateInstance().Subscribe<PointCloud2Msg>(topic, message =>
        {
            if (target.TryGetTarget(out var view) && view != null && view.isActiveAndEnabled)
                view.Receive(message);
        });
        subscribed = true;
        Status = "Waiting for " + topic;
    }

    public void Receive(PointCloud2Msg message)
    {
        lock (gate) pending = message;
    }

    private void Update()
    {
        if (mesh == null || material == null) return;
        float now = Time.unscaledTime;
        if (now >= nextRefresh)
        {
            nextRefresh = now + 1f / Mathf.Max(1, refreshHz);
            PointCloud2Msg message;
            lock (gate) { message = pending; pending = null; }
            if (message != null) ApplyFrame(message, now);
        }
        if (VisiblePoints > 0 && now - lastValidFrame > Mathf.Max(0.1f, staleSeconds))
        {
            mesh.Clear();
            VisiblePoints = 0;
            Status = "No recent valid cloud (display cleared)";
        }
        if (VisiblePoints == 0) return;
        material.SetFloat("_PointDiameter", pointDiameter);
        // Draw for every active camera, without introducing renderers/colliders into the robot hierarchy.
        Graphics.DrawMesh(mesh, DisplayMatrix, material,
            gameObject.layer, null, 0, null, ShadowCastingMode.Off, false);
    }

    private void ApplyFrame(PointCloud2Msg message, float now)
    {
        string frame = message.header?.frame_id?.TrimStart('/');
        string error = null;
        if (frame != expectedFrameId.TrimStart('/')) error = "Frame mismatch: received '" + frame + "', expected '" + expectedFrameId + "'";
        else if (RealSensePointCloudDecoder.Decode(message, coordinates, Mathf.Clamp(maxPoints, 100, 100000), positions, colors, out error))
        {
            RebuildMesh();
            lastValidFrame = now;
            Status = "Live: " + VisiblePoints + " points, frame " + frame +
                (usePreviewOffset ? " | PREVIEW OFFSET (not calibrated)" : " | Frame coordinates");
            if (!loggedFirstCloud && VisiblePoints > 0)
            {
                loggedFirstCloud = true;
                Debug.Log("RealSense point cloud: " + Status + "; displayed center " +
                    DisplayMatrix.MultiplyPoint3x4(mesh.bounds.center).ToString("F3"), this);
            }
            return;
        }
        Status = error;
        if (now >= nextWarning) { Debug.LogWarning("RealSense: " + error, this); nextWarning = now + 5; }
    }

    private void RebuildMesh()
    {
        vertices.Clear(); vertexColors.Clear(); uv.Clear(); triangles.Clear();
        for (int i = 0; i < positions.Count; i++)
        {
            int v = vertices.Count;
            for (int j = 0; j < 4; j++) { vertices.Add(positions[i]); vertexColors.Add(colors[i]); }
            uv.Add(new Vector2(-1, -1)); uv.Add(new Vector2(-1, 1));
            uv.Add(new Vector2(1, 1)); uv.Add(new Vector2(1, -1));
            triangles.Add(v); triangles.Add(v + 1); triangles.Add(v + 2);
            triangles.Add(v); triangles.Add(v + 2); triangles.Add(v + 3);
        }
        mesh.Clear();
        mesh.SetVertices(vertices); mesh.SetColors(vertexColors); mesh.SetUVs(0, uv);
        mesh.SetTriangles(triangles, 0);
        if (positions.Count > 0)
        {
            var bounds = mesh.bounds;
            bounds.Expand(0.1f);
            mesh.bounds = bounds;
        }
        VisiblePoints = positions.Count;
    }

    private void OnDisable()
    {
        lock (gate) pending = null;
        if (mesh != null) mesh.Clear();
        VisiblePoints = 0;
        Status = subscribed ? "Paused" : Status;
    }

    private void OnDestroy()
    {
        if (mesh != null) Destroy(mesh);
        if (material != null) Destroy(material);
    }
}

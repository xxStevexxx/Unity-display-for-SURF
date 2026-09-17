using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class RealSensePointCloudMenu
{
    [MenuItem("Piper/RealSense/Add Live Point Cloud")]
    public static void Add()
    {
        var existing = Object.FindFirstObjectByType<RealSensePointCloudView>();
        if (existing != null) { Selection.activeGameObject = existing.gameObject; return; }
        var root = new GameObject("RealSense Live Point Cloud");
        Undo.RegisterCreatedObjectUndo(root, "Add RealSense point cloud");
        var view = root.AddComponent<RealSensePointCloudView>();
        var baseFrame = GameObject.Find("BaseLink_Frame");
        if (baseFrame != null) view.frameOrigin = baseFrame.transform;
        Selection.activeGameObject = root;
        EditorSceneManager.MarkSceneDirty(root.scene);
    }
}

[CustomEditor(typeof(RealSensePointCloudView))]
public sealed class RealSensePointCloudViewEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        var view = (RealSensePointCloudView)target;
        EditorGUILayout.HelpBox(view.Status, MessageType.Info);
        EditorGUILayout.LabelField("Visible points", view.VisiblePoints.ToString());
        if (view.usePreviewOffset)
            EditorGUILayout.HelpBox("PREVIEW OFFSET: moved above the demo platform for visibility. This position is NOT calibrated to the real robot. Disable Use Preview Offset after calibration.", MessageType.Warning);
        EditorGUILayout.HelpBox("Display only. No colliders, surface fusion or automatic TF tracking. Match the ROS frame and calibrate Frame Origin before using real robot coordinates.", MessageType.Warning);
        if (Application.isPlaying) Repaint();
    }
}

using UnityEngine;

[ExecuteAlways]
[RequireComponent(typeof(Camera))]
public sealed class PiperGameCameraLayout : MonoBehaviour
{
    [SerializeField] private Transform target;
    [SerializeField] private Vector3 targetFallback = new(0f, 0.28f, 0.16f);
    [SerializeField] private Vector3 mainViewOffset = new(1.1f, 1.0f, -1.16f);
    [SerializeField] private float sideDistance = 1.45f;
    [SerializeField] private float orthographicSize = 0.72f;
    [SerializeField] private float insetWidth = 0.22f;
    [SerializeField] private float insetHeight = 0.24f;
    [SerializeField] private float insetPadding = 0.015f;
    [SerializeField] private bool showInsetCameras = true;

    private Camera mainCamera;
    private Camera xCamera;
    private Camera yCamera;
    private Camera zCamera;
    private GUIStyle labelStyle;

    private void OnEnable()
    {
        mainCamera = GetComponent<Camera>();
        ResolveTarget();
        EnsureInsetCameras();
        ApplyLayout();
    }

    private void OnDisable()
    {
        SetInsetActive(false);
    }

    private void LateUpdate()
    {
        ResolveTarget();
        EnsureInsetCameras();
        ApplyLayout();
    }

    private void OnGUI()
    {
        if (Event.current.type != EventType.Repaint)
            return;

        if (labelStyle == null)
        {
            labelStyle = new GUIStyle(GUI.skin.label)
            {
                alignment = TextAnchor.MiddleCenter,
                fontSize = 13,
                fontStyle = FontStyle.Bold
            };
            labelStyle.normal.textColor = Color.white;
        }

        DrawLabel(new Rect(12f, 34f, 86f, 24f), "Web");
        if (!showInsetCameras)
            return;

        DrawCameraLabel(xCamera, "X");
        DrawCameraLabel(yCamera, "Y");
        DrawCameraLabel(zCamera, "Z");
    }

    private void OnValidate()
    {
        if (mainViewOffset.sqrMagnitude < 0.04f)
            mainViewOffset = new Vector3(1.1f, 1.0f, -1.16f);
        sideDistance = Mathf.Max(0.2f, sideDistance);
        orthographicSize = Mathf.Max(0.1f, orthographicSize);
        insetWidth = Mathf.Clamp(insetWidth, 0.08f, 0.45f);
        insetHeight = Mathf.Clamp(insetHeight, 0.08f, 0.45f);
        insetPadding = Mathf.Clamp(insetPadding, 0f, 0.08f);

        if (!isActiveAndEnabled)
            return;

        mainCamera = GetComponent<Camera>();
        ApplyLayout();
    }

    private void ResolveTarget()
    {
        if (target != null)
            return;

        GameObject baseLink = GameObject.Find("base_link");
        if (baseLink != null)
        {
            target = baseLink.transform;
            return;
        }

        GameObject piper = GameObject.Find("piper");
        if (piper != null)
            target = piper.transform;
    }

    private Vector3 TargetPosition => target != null ? target.position + Vector3.up * 0.18f : targetFallback;

    private void EnsureInsetCameras()
    {
        xCamera = EnsureCamera("Camera X View", xCamera);
        yCamera = EnsureCamera("Camera Y View", yCamera);
        zCamera = EnsureCamera("Camera Z View", zCamera);
        SetInsetActive(showInsetCameras);
    }

    private Camera EnsureCamera(string cameraName, Camera existing)
    {
        if (existing != null)
            return existing;

        Transform child = transform.Find(cameraName);
        if (child == null)
        {
            GameObject cameraObject = new(cameraName);
            cameraObject.transform.SetParent(transform, false);
            child = cameraObject.transform;
        }

        Camera camera = child.GetComponent<Camera>();
        if (camera == null)
            camera = child.gameObject.AddComponent<Camera>();

        camera.enabled = showInsetCameras;
        camera.orthographic = true;
        camera.orthographicSize = orthographicSize;
        camera.nearClipPlane = 0.01f;
        camera.farClipPlane = 100f;
        camera.clearFlags = CameraClearFlags.SolidColor;
        camera.backgroundColor = new Color(0.95f, 0.97f, 1f, 1f);
        camera.depth = mainCamera != null ? mainCamera.depth + 1f : 1f;
        camera.cullingMask = mainCamera != null ? mainCamera.cullingMask : -1;
        return camera;
    }

    private void ApplyLayout()
    {
        if (mainCamera == null)
            return;

        Vector3 center = TargetPosition;
        mainCamera.orthographic = true;
        mainCamera.orthographicSize = orthographicSize;
        mainCamera.rect = new Rect(0f, 0f, 1f, 1f);
        mainCamera.depth = 0f;
        mainCamera.clearFlags = CameraClearFlags.Skybox;
        transform.position = center + mainViewOffset;
        transform.rotation = Quaternion.LookRotation(center - transform.position, Vector3.up);

        if (!showInsetCameras)
            return;

        ApplyInsetCamera(xCamera, center + Vector3.right * sideDistance, center, 0);
        ApplyInsetCamera(yCamera, center + Vector3.up * sideDistance, center, 1);
        ApplyInsetCamera(zCamera, center + Vector3.forward * sideDistance, center, 2);
    }

    private void ApplyInsetCamera(Camera camera, Vector3 position, Vector3 center, int index)
    {
        if (camera == null)
            return;

        camera.transform.position = position;
        Vector3 up = index == 1 ? Vector3.forward : Vector3.up;
        camera.transform.rotation = Quaternion.LookRotation(center - position, up);
        camera.orthographic = true;
        camera.orthographicSize = orthographicSize;
        camera.rect = new Rect(
            1f - insetPadding - insetWidth,
            1f - insetPadding - insetHeight - index * (insetHeight + insetPadding),
            insetWidth,
            insetHeight);
    }

    private void DrawCameraLabel(Camera camera, string label)
    {
        if (camera == null || !camera.enabled)
            return;

        Rect rect = camera.rect;
        float x = rect.xMin * Screen.width + 8f;
        float y = (1f - rect.yMax) * Screen.height + 8f;
        DrawLabel(new Rect(x, y, 34f, 22f), label);
    }

    private void DrawLabel(Rect rect, string label)
    {
        Color previous = GUI.color;
        GUI.color = new Color(0f, 0f, 0f, 0.48f);
        GUI.DrawTexture(rect, Texture2D.whiteTexture);
        GUI.color = previous;
        GUI.Label(rect, label, labelStyle);
    }

    private void SetInsetActive(bool active)
    {
        if (xCamera != null)
            xCamera.enabled = active;
        if (yCamera != null)
            yCamera.enabled = active;
        if (zCamera != null)
            zCamera.enabled = active;
    }
}

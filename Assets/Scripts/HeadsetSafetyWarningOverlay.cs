using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

[DefaultExecutionOrder(200)]
public sealed class HeadsetSafetyWarningOverlay : MonoBehaviour
{
    [Header("Headset UI")]
    [SerializeField] private Camera headsetCamera;
    [SerializeField] private Vector2 canvasResolution = new Vector2(1280f, 720f);
    [SerializeField] private float headsetDistanceMeters = 0.75f;
    [SerializeField] private bool autoScaleCanvasToCamera = true;
    [SerializeField] private float worldCanvasScale = 0.00125f;
    [SerializeField] private int overlayLayer = 5;
    [SerializeField] private float frameThickness = 5f;
    [SerializeField] private float framePaddingPixels = 10f;
    [SerializeField] private bool hideWhenObjectIsOutsideView = true;

    [Header("Zones")]
    [SerializeField] private bool autoFindZones = true;
    [SerializeField] private SafetyTemperatureZone[] zones;

    private readonly Dictionary<SafetyTemperatureZone, ZoneUi> zoneUis = new Dictionary<SafetyTemperatureZone, ZoneUi>();
    private RectTransform canvasRect;
    private Transform uiRoot;
    private Font uiFont;

    private void Awake()
    {
        ResolveCamera();
        BuildCanvas();
        BuildLegend();
    }

    private void LateUpdate()
    {
        ResolveCamera();
        AttachCanvasToCamera();
        RefreshZones();
        UpdateZoneFrames();
    }

    public void SetZones(SafetyTemperatureZone[] nextZones)
    {
        zones = nextZones;
        RefreshZones();
    }

    private void ResolveCamera()
    {
        if (headsetCamera != null)
            return;

        headsetCamera = Camera.main;
        if (headsetCamera == null)
            headsetCamera = Object.FindAnyObjectByType<Camera>();
    }

    private void BuildCanvas()
    {
        uiFont = ResolveFont();

        GameObject canvasObject = new GameObject("Headset Safety Warning Canvas");
        canvasObject.transform.SetParent(transform, false);

        Canvas canvas = canvasObject.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
        canvas.overrideSorting = true;
        canvas.sortingOrder = 2000;
        canvasObject.layer = overlayLayer;

        canvasObject.AddComponent<CanvasScaler>();
        canvasObject.AddComponent<GraphicRaycaster>();

        canvasRect = canvasObject.GetComponent<RectTransform>();
        canvasRect.sizeDelta = canvasResolution;

        uiRoot = canvasObject.transform;
        AttachCanvasToCamera();
    }

    private void AttachCanvasToCamera()
    {
        if (canvasRect == null || headsetCamera == null)
            return;

        Canvas canvas = canvasRect.GetComponent<Canvas>();
        canvas.worldCamera = headsetCamera;

        Transform canvasTransform = canvasRect.transform;
        canvasTransform.SetParent(headsetCamera.transform, false);
        canvasTransform.localPosition = new Vector3(0f, 0f, headsetDistanceMeters);
        canvasTransform.localRotation = Quaternion.identity;

        Vector2 canvasSize = CalculateCanvasSize();
        canvasRect.sizeDelta = canvasSize;
        canvasTransform.localScale = Vector3.one * CalculateCanvasScale(canvasSize.y);

        ConfigureCameraLayerMasks();
    }

    private void RefreshZones()
    {
        if (autoFindZones || zones == null || zones.Length == 0)
            zones = Object.FindObjectsByType<SafetyTemperatureZone>();

        if (zones == null)
            return;

        for (int i = 0; i < zones.Length; i++)
        {
            SafetyTemperatureZone zone = zones[i];
            if (zone == null || zoneUis.ContainsKey(zone))
                continue;

            zoneUis.Add(zone, CreateZoneUi(zone.ZoneName));
        }
    }

    private void UpdateZoneFrames()
    {
        if (headsetCamera == null || zones == null)
            return;

        for (int i = 0; i < zones.Length; i++)
        {
            SafetyTemperatureZone zone = zones[i];
            if (zone == null || !zoneUis.TryGetValue(zone, out ZoneUi ui))
                continue;

            if (!TryGetScreenRect(zone.GetWorldBounds(), out Rect rect))
            {
                ui.SetVisible(false);
                continue;
            }

            ui.SetVisible(true);
            ui.SetRect(rect);
            ui.SetColor(GetWarningColor(zone.Level));
            ui.SetText(GetZoneLabel(zone));
        }
    }

    private static string GetZoneLabel(SafetyTemperatureZone zone)
    {
        string levelName = SafetyTemperatureZone.DisplayNameForLevel(zone.Level);
        if (zone.Level == SafetyTemperatureLevel.Critical)
            levelName = "Critical Alarm";

        return $"{zone.ZoneName} [{levelName}]\nAvg {zone.AverageTemperatureC:F1} C";
    }

    private static Color GetWarningColor(SafetyTemperatureLevel level)
    {
        Color color = SafetyTemperatureZone.ColorForLevel(level);
        if (level != SafetyTemperatureLevel.Critical)
            return color;

        float pulse = Mathf.PingPong(Time.unscaledTime * 3.2f, 1f);
        color.a = Mathf.Lerp(0.35f, 1f, pulse);
        return color;
    }

    private bool TryGetScreenRect(Bounds worldBounds, out Rect localRect)
    {
        localRect = default;

        Vector3 min = worldBounds.min;
        Vector3 max = worldBounds.max;
        Vector3[] corners =
        {
            new Vector3(min.x, min.y, min.z),
            new Vector3(max.x, min.y, min.z),
            new Vector3(min.x, max.y, min.z),
            new Vector3(max.x, max.y, min.z),
            new Vector3(min.x, min.y, max.z),
            new Vector3(max.x, min.y, max.z),
            new Vector3(min.x, max.y, max.z),
            new Vector3(max.x, max.y, max.z)
        };

        bool hasPoint = false;
        float minX = float.PositiveInfinity;
        float minY = float.PositiveInfinity;
        float maxX = float.NegativeInfinity;
        float maxY = float.NegativeInfinity;

        for (int i = 0; i < corners.Length; i++)
        {
            Vector3 viewport = headsetCamera.WorldToViewportPoint(corners[i]);
            if (viewport.z <= headsetCamera.nearClipPlane)
                continue;

            hasPoint = true;
            minX = Mathf.Min(minX, viewport.x);
            minY = Mathf.Min(minY, viewport.y);
            maxX = Mathf.Max(maxX, viewport.x);
            maxY = Mathf.Max(maxY, viewport.y);
        }

        if (!hasPoint)
            return false;

        if (hideWhenObjectIsOutsideView && (maxX < 0f || minX > 1f || maxY < 0f || minY > 1f))
            return false;

        minX = Mathf.Clamp01(minX);
        maxX = Mathf.Clamp01(maxX);
        minY = Mathf.Clamp01(minY);
        maxY = Mathf.Clamp01(maxY);

        Vector2 canvasSize = canvasRect != null ? canvasRect.sizeDelta : CalculateCanvasSize();
        float x = (minX - 0.5f) * canvasSize.x - framePaddingPixels;
        float y = (minY - 0.5f) * canvasSize.y - framePaddingPixels;
        float width = (maxX - minX) * canvasSize.x + framePaddingPixels * 2f;
        float height = (maxY - minY) * canvasSize.y + framePaddingPixels * 2f;

        if (width < 8f || height < 8f)
            return false;

        localRect = new Rect(x, y, width, height);
        return true;
    }

    private ZoneUi CreateZoneUi(string zoneName)
    {
        GameObject rootObject = new GameObject($"Safety Frame - {zoneName}");
        rootObject.transform.SetParent(uiRoot, false);
        RectTransform root = rootObject.AddComponent<RectTransform>();
        root.anchorMin = new Vector2(0.5f, 0.5f);
        root.anchorMax = new Vector2(0.5f, 0.5f);
        root.pivot = new Vector2(0f, 0f);

        Image top = CreateFrameEdge(root, "Top");
        Image bottom = CreateFrameEdge(root, "Bottom");
        Image left = CreateFrameEdge(root, "Left");
        Image right = CreateFrameEdge(root, "Right");

        ConfigureHorizontalEdge(top.rectTransform, true);
        ConfigureHorizontalEdge(bottom.rectTransform, false);
        ConfigureVerticalEdge(left.rectTransform, true);
        ConfigureVerticalEdge(right.rectTransform, false);

        Text label = CreateText(root, "Label", string.Empty, 22, TextAnchor.MiddleLeft, Color.white);
        RectTransform labelRect = label.rectTransform;
        labelRect.anchorMin = new Vector2(0f, 1f);
        labelRect.anchorMax = new Vector2(1f, 1f);
        labelRect.pivot = new Vector2(0f, 0f);
        labelRect.anchoredPosition = new Vector2(0f, 8f);
        labelRect.sizeDelta = new Vector2(0f, 52f);
        SetLayerRecursively(rootObject, overlayLayer);

        return new ZoneUi(root, label, top, bottom, left, right);
    }

    private Image CreateFrameEdge(RectTransform parent, string edgeName)
    {
        GameObject edgeObject = new GameObject(edgeName);
        edgeObject.transform.SetParent(parent, false);
        return edgeObject.AddComponent<Image>();
    }

    private void ConfigureHorizontalEdge(RectTransform rect, bool top)
    {
        rect.anchorMin = top ? new Vector2(0f, 1f) : new Vector2(0f, 0f);
        rect.anchorMax = top ? new Vector2(1f, 1f) : new Vector2(1f, 0f);
        rect.pivot = top ? new Vector2(0.5f, 1f) : new Vector2(0.5f, 0f);
        rect.anchoredPosition = Vector2.zero;
        rect.sizeDelta = new Vector2(0f, frameThickness);
    }

    private void ConfigureVerticalEdge(RectTransform rect, bool left)
    {
        rect.anchorMin = left ? new Vector2(0f, 0f) : new Vector2(1f, 0f);
        rect.anchorMax = left ? new Vector2(0f, 1f) : new Vector2(1f, 1f);
        rect.pivot = left ? new Vector2(0f, 0.5f) : new Vector2(1f, 0.5f);
        rect.anchoredPosition = Vector2.zero;
        rect.sizeDelta = new Vector2(frameThickness, 0f);
    }

    private void BuildLegend()
    {
        GameObject panelObject = new GameObject("Temperature Legend");
        panelObject.transform.SetParent(uiRoot, false);
        RectTransform panel = panelObject.AddComponent<RectTransform>();
        panel.anchorMin = new Vector2(0f, 1f);
        panel.anchorMax = new Vector2(0f, 1f);
        panel.pivot = new Vector2(0f, 1f);
        panel.anchoredPosition = new Vector2(24f, -24f);
        panel.sizeDelta = new Vector2(330f, 150f);

        Image background = panelObject.AddComponent<Image>();
        background.color = new Color(0f, 0f, 0f, 0.62f);

        Text title = CreateText(panel, "Title", "Temperature range", 22, TextAnchor.MiddleLeft, Color.white);
        RectTransform titleRect = title.rectTransform;
        titleRect.anchorMin = new Vector2(0f, 1f);
        titleRect.anchorMax = new Vector2(1f, 1f);
        titleRect.pivot = new Vector2(0f, 1f);
        titleRect.anchoredPosition = new Vector2(16f, -10f);
        titleRect.sizeDelta = new Vector2(-32f, 28f);

        CreateLegendRow(panel, 0, SafetyTemperatureLevel.Safe, "<40 C   Safe / Normal");
        CreateLegendRow(panel, 1, SafetyTemperatureLevel.Caution, "40-60 C Caution / Warn");
        CreateLegendRow(panel, 2, SafetyTemperatureLevel.Danger, "60-80 C Danger / Block");
        CreateLegendRow(panel, 3, SafetyTemperatureLevel.Critical, ">=80 C  Critical / Stop");
        SetLayerRecursively(panelObject, overlayLayer);
    }

    private void CreateLegendRow(RectTransform parent, int index, SafetyTemperatureLevel level, string text)
    {
        float y = -47f - index * 25f;

        GameObject swatchObject = new GameObject($"Legend {level} Swatch");
        swatchObject.transform.SetParent(parent, false);
        RectTransform swatch = swatchObject.AddComponent<RectTransform>();
        swatch.anchorMin = new Vector2(0f, 1f);
        swatch.anchorMax = new Vector2(0f, 1f);
        swatch.pivot = new Vector2(0f, 1f);
        swatch.anchoredPosition = new Vector2(18f, y);
        swatch.sizeDelta = new Vector2(28f, 18f);

        Image swatchImage = swatchObject.AddComponent<Image>();
        swatchImage.color = SafetyTemperatureZone.ColorForLevel(level);

        Text rowText = CreateText(parent, $"Legend {level} Text", text, 18, TextAnchor.MiddleLeft, Color.white);
        RectTransform rowRect = rowText.rectTransform;
        rowRect.anchorMin = new Vector2(0f, 1f);
        rowRect.anchorMax = new Vector2(1f, 1f);
        rowRect.pivot = new Vector2(0f, 1f);
        rowRect.anchoredPosition = new Vector2(56f, y + 1f);
        rowRect.sizeDelta = new Vector2(-70f, 22f);
    }

    private Text CreateText(RectTransform parent, string objectName, string value, int fontSize, TextAnchor alignment, Color color)
    {
        GameObject textObject = new GameObject(objectName);
        textObject.transform.SetParent(parent, false);
        Text text = textObject.AddComponent<Text>();
        text.text = value;
        text.font = uiFont;
        text.fontSize = fontSize;
        text.fontStyle = FontStyle.Bold;
        text.alignment = alignment;
        text.color = color;
        text.horizontalOverflow = HorizontalWrapMode.Overflow;
        text.verticalOverflow = VerticalWrapMode.Overflow;
        return text;
    }

    private static Font ResolveFont()
    {
        Font legacyFont = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        if (legacyFont != null)
            return legacyFont;

        return Resources.GetBuiltinResource<Font>("Arial.ttf");
    }

    private Vector2 CalculateCanvasSize()
    {
        float heightPixels = Mathf.Max(1f, canvasResolution.y);
        float fallbackAspect = canvasResolution.y > 0f ? canvasResolution.x / canvasResolution.y : 16f / 9f;
        float aspect = headsetCamera != null && headsetCamera.aspect > 0.01f
            ? headsetCamera.aspect
            : fallbackAspect;
        return new Vector2(heightPixels * aspect, heightPixels);
    }

    private float CalculateCanvasScale(float heightPixels)
    {
        if (!autoScaleCanvasToCamera || headsetCamera == null)
            return Mathf.Max(0.0001f, worldCanvasScale);

        if (headsetCamera.orthographic)
            return (headsetCamera.orthographicSize * 2f) / Mathf.Max(1f, heightPixels);

        float visibleHeight = 2f * headsetDistanceMeters * Mathf.Tan(headsetCamera.fieldOfView * 0.5f * Mathf.Deg2Rad);
        return visibleHeight / Mathf.Max(1f, heightPixels);
    }

    private void ConfigureCameraLayerMasks()
    {
        if (overlayLayer < 0 || overlayLayer > 31 || headsetCamera == null)
            return;

        int mask = 1 << overlayLayer;
        headsetCamera.cullingMask |= mask;

        Camera[] cameras = Object.FindObjectsByType<Camera>();
        for (int i = 0; i < cameras.Length; i++)
        {
            Camera camera = cameras[i];
            if (camera == null || camera == headsetCamera)
                continue;

            if (camera.transform.IsChildOf(headsetCamera.transform))
                camera.cullingMask &= ~mask;
        }
    }

    private static void SetLayerRecursively(GameObject target, int layer)
    {
        if (target == null || layer < 0 || layer > 31)
            return;

        target.layer = layer;
        for (int i = 0; i < target.transform.childCount; i++)
            SetLayerRecursively(target.transform.GetChild(i).gameObject, layer);
    }

    private sealed class ZoneUi
    {
        private readonly RectTransform root;
        private readonly Text label;
        private readonly Image[] edges;

        public ZoneUi(RectTransform root, Text label, params Image[] edges)
        {
            this.root = root;
            this.label = label;
            this.edges = edges;
        }

        public void SetVisible(bool visible)
        {
            if (root.gameObject.activeSelf != visible)
                root.gameObject.SetActive(visible);
        }

        public void SetRect(Rect rect)
        {
            root.anchoredPosition = new Vector2(rect.xMin, rect.yMin);
            root.sizeDelta = new Vector2(rect.width, rect.height);
        }

        public void SetColor(Color color)
        {
            for (int i = 0; i < edges.Length; i++)
                edges[i].color = color;

            label.color = color;
        }

        public void SetText(string text)
        {
            label.text = text;
        }
    }
}

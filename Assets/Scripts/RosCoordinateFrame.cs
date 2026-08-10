using UnityEngine;

[ExecuteAlways]
public sealed class RosCoordinateFrame : MonoBehaviour
{
    [SerializeField] private float axisLength = 0.45f;
    [SerializeField] private float shaftRadius = 0.012f;
    [SerializeField] private float headLength = 0.075f;
    [SerializeField] private float headRadius = 0.035f;
    [SerializeField] private float labelSize = 0.055f;

    private static readonly Color RosXColor = new(0.9f, 0.05f, 0.05f, 1f);
    private static readonly Color RosYColor = new(0.05f, 0.7f, 0.15f, 1f);
    private static readonly Color RosZColor = new(0.05f, 0.25f, 0.95f, 1f);

    private void OnEnable()
    {
        Rebuild();
    }

    private void OnValidate()
    {
        axisLength = Mathf.Max(0.1f, axisLength);
        shaftRadius = Mathf.Max(0.002f, shaftRadius);
        headLength = Mathf.Clamp(headLength, 0.02f, axisLength * 0.45f);
        headRadius = Mathf.Max(shaftRadius * 1.5f, headRadius);
        labelSize = Mathf.Max(0.02f, labelSize);
    }

    [ContextMenu("Rebuild ROS Coordinate Frame")]
    public void Rebuild()
    {
        ClearGeneratedChildren();

        BuildAxis("ROS Axis X Forward", "X", Vector3.forward, RosXColor);
        BuildAxis("ROS Axis Y Left", "Y", Vector3.left, RosYColor);
        BuildAxis("ROS Axis Z Up", "Z", Vector3.up, RosZColor);
    }

    private void ClearGeneratedChildren()
    {
        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            Transform child = transform.GetChild(i);
            string childName = child.name;
            if (!childName.StartsWith("ROS Axis") && childName != "AxisX" && childName != "AxisY" && childName != "AxisZ")
                continue;

            if (Application.isPlaying)
                Destroy(child.gameObject);
            else
                DestroyImmediate(child.gameObject);
        }
    }

    private void BuildAxis(string objectName, string label, Vector3 direction, Color color)
    {
        Vector3 normalized = direction.normalized;
        Material material = CreateMaterial(objectName + " Material", color);

        GameObject root = new(objectName);
        root.transform.SetParent(transform, false);

        float shaftLength = Mathf.Max(0.01f, axisLength - headLength);
        GameObject shaft = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        shaft.name = "Shaft";
        shaft.transform.SetParent(root.transform, false);
        shaft.transform.localScale = new Vector3(shaftRadius, shaftLength * 0.5f, shaftRadius);
        shaft.transform.localPosition = normalized * (shaftLength * 0.5f);
        shaft.transform.localRotation = Quaternion.FromToRotation(Vector3.up, normalized);
        ApplyMaterial(shaft, material);
        RemoveCollider(shaft);

        GameObject head = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        head.name = "Head";
        head.transform.SetParent(root.transform, false);
        head.transform.localScale = new Vector3(headRadius, headLength * 0.5f, headRadius);
        head.transform.localPosition = normalized * (shaftLength + headLength * 0.5f);
        head.transform.localRotation = Quaternion.FromToRotation(Vector3.up, normalized);
        ApplyMaterial(head, material);
        RemoveCollider(head);

        GameObject labelObject = new("Label");
        labelObject.transform.SetParent(root.transform, false);
        labelObject.transform.localPosition = normalized * (axisLength + labelSize * 0.9f);
        TextMesh text = labelObject.AddComponent<TextMesh>();
        text.text = label;
        text.characterSize = labelSize;
        text.anchor = TextAnchor.MiddleCenter;
        text.alignment = TextAlignment.Center;
        text.color = color;
    }

    private static Material CreateMaterial(string materialName, Color color)
    {
        Shader shader = Shader.Find("Universal Render Pipeline/Lit");
        if (shader == null)
            shader = Shader.Find("Standard");

        Material material = new(shader)
        {
            name = materialName,
            color = color,
            hideFlags = HideFlags.DontSave
        };
        return material;
    }

    private static void ApplyMaterial(GameObject target, Material material)
    {
        if (target.TryGetComponent(out Renderer renderer))
            renderer.sharedMaterial = material;
    }

    private static void RemoveCollider(GameObject target)
    {
        if (!target.TryGetComponent(out Collider collider))
            return;

        if (Application.isPlaying)
            Destroy(collider);
        else
            DestroyImmediate(collider);
    }
}

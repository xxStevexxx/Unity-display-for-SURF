using System.Collections.Generic;
using UnityEngine;

[DefaultExecutionOrder(-20)]
public sealed class SafetyWarningDemoBootstrapper : MonoBehaviour
{
    [SerializeField] private bool createDemoObjectsIfNoZones = true;

    private static bool installed;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void InstallAfterSceneLoad()
    {
        if (installed || Object.FindAnyObjectByType<HeadsetSafetyWarningOverlay>() != null)
            return;

        installed = true;
        GameObject host = new GameObject("Safety Warning Runtime");
        DontDestroyOnLoad(host);

        SafetyWarningDemoBootstrapper bootstrapper = host.AddComponent<SafetyWarningDemoBootstrapper>();
        bootstrapper.Install();
    }

    private void Start()
    {
        Install();
    }

    private void Install()
    {
        SafetyTemperatureZone[] zones = Object.FindObjectsByType<SafetyTemperatureZone>();
        if ((zones == null || zones.Length == 0) && createDemoObjectsIfNoZones)
            zones = CreateDemoTemperatureZones();

        HeadsetSafetyWarningOverlay overlay = Object.FindAnyObjectByType<HeadsetSafetyWarningOverlay>();
        if (overlay == null)
            overlay = gameObject.AddComponent<HeadsetSafetyWarningOverlay>();

        overlay.SetZones(zones);
    }

    private SafetyTemperatureZone[] CreateDemoTemperatureZones()
    {
        List<SafetyTemperatureZone> zones = new List<SafetyTemperatureZone>();

        AddExistingTargetIfAvailable(zones, "Detected_Object", "Detected Object", 25f, 2.2f, 0.45f);
        AddExistingTargetIfAvailable(zones, "Converted_Target_Marker", "Converted Target", 48f, 3.0f, 0.55f);

        Vector3 center = ResolveRobotCenter();
        Quaternion rotation = Quaternion.identity;
        CreateDemoObject(zones, "Simulated Safe Workpiece", center + new Vector3(-0.24f, 0.12f, 0.18f), rotation, 25f, 2.0f, 0.4f);
        CreateDemoObject(zones, "Simulated Caution Equipment", center + new Vector3(0.26f, 0.13f, 0.15f), rotation, 50f, 4.0f, 0.5f);
        CreateDemoObject(zones, "Simulated Danger Surface", center + new Vector3(0.02f, 0.13f, -0.26f), rotation, 70f, 3.5f, 0.65f);
        CreateDemoObject(zones, "Simulated Critical Furnace", center + new Vector3(-0.25f, 0.13f, -0.25f), rotation, 86f, 3.5f, 0.75f);

        return zones.ToArray();
    }

    private void AddExistingTargetIfAvailable(
        List<SafetyTemperatureZone> zones,
        string objectName,
        string displayName,
        float baseTemperatureC,
        float variationC,
        float speed)
    {
        GameObject target = GameObject.Find(objectName);
        if (target == null || target.GetComponentInChildren<Renderer>() == null)
            return;

        SafetyTemperatureZone zone = target.GetComponent<SafetyTemperatureZone>();
        if (zone == null)
            zone = target.AddComponent<SafetyTemperatureZone>();

        zone.Configure(displayName, baseTemperatureC, variationC, speed);
        zones.Add(zone);
    }

    private void CreateDemoObject(
        List<SafetyTemperatureZone> zones,
        string objectName,
        Vector3 position,
        Quaternion rotation,
        float baseTemperatureC,
        float variationC,
        float speed)
    {
        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.name = objectName;
        cube.transform.position = position;
        cube.transform.rotation = rotation;
        cube.transform.localScale = new Vector3(0.14f, 0.12f, 0.14f);

        Renderer renderer = cube.GetComponent<Renderer>();
        if (renderer != null)
            renderer.material = CreateNeutralDemoMaterial();

        SafetyTemperatureZone zone = cube.AddComponent<SafetyTemperatureZone>();
        zone.Configure(objectName, baseTemperatureC, variationC, speed);
        zones.Add(zone);
    }

    private static Material CreateNeutralDemoMaterial()
    {
        Shader shader = Shader.Find("Universal Render Pipeline/Lit");
        if (shader == null)
            shader = Shader.Find("Standard");

        Material material = new Material(shader);
        material.name = "Neutral Temperature Demo Material";
        material.color = new Color(0.72f, 0.74f, 0.76f, 1f);
        return material;
    }

    private static Vector3 ResolveRobotCenter()
    {
        GameObject baseLink = GameObject.Find("base_link");
        if (baseLink != null)
            return baseLink.transform.position;

        GameObject piper = GameObject.Find("piper");
        if (piper != null)
            return piper.transform.position;

        return Vector3.zero;
    }
}

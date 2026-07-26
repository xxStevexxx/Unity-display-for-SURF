using UnityEngine;

[DisallowMultipleComponent]
public sealed class SafetyTemperatureZone : MonoBehaviour
{
    public const float T1C = 40f;
    public const float T2C = 65f;
    public const float T3C = 120f;

    [Header("Zone")]
    [SerializeField] private string zoneName = "Environment Object";
    [SerializeField] private Renderer[] zoneRenderers;
    [SerializeField] private Collider[] zoneColliders;
    [SerializeField] private Vector3 fallbackBoundsSize = new Vector3(0.16f, 0.16f, 0.16f);
    [SerializeField] private float boundsPadding = 0.02f;

    [Header("Temperature Simulation")]
    [SerializeField] private bool simulateTemperature = true;
    [SerializeField] private float baseTemperatureC = 25f;
    [SerializeField] private float timeVariationC = 2.5f;
    [SerializeField] private float spatialVariationC = 1.5f;
    [SerializeField] private float noiseC = 0.4f;
    [SerializeField] private float simulationSpeed = 0.45f;
    [SerializeField] private int sampleCount = 9;

    private float currentAverageTemperatureC;
    private float nextSampleTime;
    private float phase;

    public string ZoneName => string.IsNullOrWhiteSpace(zoneName) ? name : zoneName;
    public float AverageTemperatureC => currentAverageTemperatureC;
    public SafetyTemperatureLevel Level => Classify(currentAverageTemperatureC);

    private void Awake()
    {
        phase = Mathf.Abs(
            transform.position.x * 12.9898f +
            transform.position.y * 78.233f +
            transform.position.z * 37.719f +
            name.Length * 0.618f);
        CacheBoundsSourcesIfNeeded();
        SampleTemperature(true);
    }

    private void Update()
    {
        if (!simulateTemperature || Time.time < nextSampleTime)
            return;

        SampleTemperature(false);
    }

    public void Configure(string displayName, float baseTemperature, float variation, float speed)
    {
        zoneName = displayName;
        baseTemperatureC = baseTemperature;
        timeVariationC = variation;
        simulationSpeed = speed;
        simulateTemperature = true;
        SampleTemperature(true);
    }

    public void SetManualTemperature(float temperatureC)
    {
        simulateTemperature = false;
        currentAverageTemperatureC = temperatureC;
    }

    public Bounds GetWorldBounds()
    {
        CacheBoundsSourcesIfNeeded();

        bool hasBounds = false;
        Bounds bounds = new Bounds(transform.position, fallbackBoundsSize);

        for (int i = 0; i < zoneRenderers.Length; i++)
        {
            Renderer zoneRenderer = zoneRenderers[i];
            if (zoneRenderer == null || !zoneRenderer.enabled)
                continue;

            if (!hasBounds)
            {
                bounds = zoneRenderer.bounds;
                hasBounds = true;
            }
            else
            {
                bounds.Encapsulate(zoneRenderer.bounds);
            }
        }

        if (!hasBounds)
        {
            for (int i = 0; i < zoneColliders.Length; i++)
            {
                Collider zoneCollider = zoneColliders[i];
                if (zoneCollider == null || !zoneCollider.enabled)
                    continue;

                if (!hasBounds)
                {
                    bounds = zoneCollider.bounds;
                    hasBounds = true;
                }
                else
                {
                    bounds.Encapsulate(zoneCollider.bounds);
                }
            }
        }

        if (!hasBounds)
            bounds = new Bounds(transform.position, fallbackBoundsSize);

        bounds.Expand(boundsPadding);
        return bounds;
    }

    public static SafetyTemperatureLevel Classify(float temperatureC)
    {
        if (temperatureC < T1C)
            return SafetyTemperatureLevel.Safe;
        if (temperatureC < T2C)
            return SafetyTemperatureLevel.Caution;
        if (temperatureC < T3C)
            return SafetyTemperatureLevel.Danger;
        return SafetyTemperatureLevel.Critical;
    }

    public static Color ColorForLevel(SafetyTemperatureLevel level)
    {
        switch (level)
        {
            case SafetyTemperatureLevel.Safe:
                return new Color(0.1f, 0.9f, 0.25f, 0.95f);
            case SafetyTemperatureLevel.Caution:
                return new Color(1f, 0.82f, 0.08f, 0.95f);
            case SafetyTemperatureLevel.Danger:
                return new Color(1f, 0.08f, 0.06f, 0.95f);
            default:
                return new Color(1f, 0f, 0f, 1f);
        }
    }

    public static string DisplayNameForLevel(SafetyTemperatureLevel level)
    {
        switch (level)
        {
            case SafetyTemperatureLevel.Safe:
                return "Safe";
            case SafetyTemperatureLevel.Caution:
                return "Caution";
            case SafetyTemperatureLevel.Danger:
                return "Danger";
            default:
                return "Critical";
        }
    }

    private void CacheBoundsSourcesIfNeeded()
    {
        if (zoneRenderers == null || zoneRenderers.Length == 0)
            zoneRenderers = GetComponentsInChildren<Renderer>();
        if (zoneColliders == null || zoneColliders.Length == 0)
            zoneColliders = GetComponentsInChildren<Collider>();
    }

    private void SampleTemperature(bool force)
    {
        if (!force && Time.time < nextSampleTime)
            return;

        nextSampleTime = Time.time + 0.25f;
        int count = Mathf.Max(1, sampleCount);
        float sum = 0f;

        for (int i = 0; i < count; i++)
        {
            float normalized = count == 1 ? 0.5f : i / (float)(count - 1);
            float spatial = Mathf.Lerp(-spatialVariationC, spatialVariationC, normalized);
            float wave = Mathf.Sin(Time.time * simulationSpeed + phase + i * 0.37f) * timeVariationC;
            float noise = (Mathf.PerlinNoise(phase + i * 0.23f, Time.time * 0.2f) - 0.5f) * noiseC;
            sum += baseTemperatureC + spatial + wave + noise;
        }

        currentAverageTemperatureC = sum / count;
    }
}

public enum SafetyTemperatureLevel
{
    Safe,
    Caution,
    Danger,
    Critical
}

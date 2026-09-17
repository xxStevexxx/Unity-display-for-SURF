using System;
using UnityEngine;
using UnityEngine.Rendering;

// Renders only the visible model surfaces into a transparent UI texture.
public sealed class TemperatureProjectionMask : IDisposable
{
    private static readonly int ViewProjectionId = Shader.PropertyToID("_TemperatureViewProjection");
    private static readonly int TintId = Shader.PropertyToID("_TemperatureTint");
    private readonly CommandBuffer commands = new() { name = "Temperature UI silhouette" };
    private readonly Plane[] frustumPlanes = new Plane[6];
    private Material material;
    private RenderTexture texture;
    private Renderer[] sceneRenderers;
    private float nextRendererRefresh;

    public RenderTexture Texture => texture;

    public void Render(Camera camera, SafetyTemperatureZone[] zones, float opacity,
        Func<SafetyTemperatureLevel, Color> warningColor)
    {
        if (camera == null)
            return;
        if (material == null)
        {
            Shader shader = Resources.Load<Shader>("TemperatureProjectionMask");
            if (shader == null || !shader.isSupported)
                return;
            material = new Material(shader) { hideFlags = HideFlags.HideAndDontSave };
        }

        EnsureTexture(Mathf.Max(1, camera.pixelWidth), Mathf.Max(1, camera.pixelHeight));
        if (sceneRenderers == null || Time.unscaledTime >= nextRendererRefresh)
        {
            sceneRenderers = UnityEngine.Object.FindObjectsByType<Renderer>();
            nextRendererRefresh = Time.unscaledTime + 0.5f;
        }

        GeometryUtility.CalculateFrustumPlanes(camera, frustumPlanes);
        commands.Clear();
        commands.SetRenderTarget(texture);
        commands.SetViewport(new Rect(0, 0, texture.width, texture.height));
        commands.ClearRenderTarget(true, true, Color.clear);
        commands.SetGlobalMatrix(ViewProjectionId,
            GL.GetGPUProjectionMatrix(camera.projectionMatrix, true) * camera.worldToCameraMatrix);

        // Record scene depth so the UI cannot tint a finger in front of a cube.
        foreach (Renderer renderer in sceneRenderers)
        {
            if (CanDraw(renderer, camera))
                DrawSubmeshes(renderer, 0, true);
        }

        if (zones != null)
        {
            foreach (SafetyTemperatureZone zone in zones)
            {
                if (zone == null || !zone.isActiveAndEnabled)
                    continue;
                Color tint = warningColor(zone.Level);
                tint.a *= opacity;
                commands.SetGlobalColor(TintId, tint);
                foreach (Renderer renderer in zone.GetComponentsInChildren<Renderer>())
                {
                    if (CanDraw(renderer, camera))
                        DrawSubmeshes(renderer, 1, false);
                }
            }
        }

        RenderTexture previousTarget = RenderTexture.active;
        Graphics.ExecuteCommandBuffer(commands);
        RenderTexture.active = previousTarget;
    }

    private bool CanDraw(Renderer renderer, Camera camera)
    {
        return renderer != null && renderer.enabled && !renderer.forceRenderingOff &&
            renderer.gameObject.activeInHierarchy &&
            renderer.shadowCastingMode != ShadowCastingMode.ShadowsOnly &&
            (renderer is MeshRenderer || renderer is SkinnedMeshRenderer) &&
            (camera.cullingMask & (1 << renderer.gameObject.layer)) != 0 &&
            GeometryUtility.TestPlanesAABB(frustumPlanes, renderer.bounds);
    }

    private void DrawSubmeshes(Renderer renderer, int pass, bool opaqueOnly)
    {
        Mesh mesh;
        if (renderer is SkinnedMeshRenderer skinned)
            mesh = skinned.sharedMesh;
        else if (renderer.TryGetComponent<MeshFilter>(out var meshFilter) && meshFilter != null)
            mesh = meshFilter.sharedMesh;
        else
            return;
        if (mesh == null)
            return;
        Material[] materials = renderer.sharedMaterials;
        for (int i = 0; i < mesh.subMeshCount && i < materials.Length; i++)
        {
            if (materials[i] == null || (opaqueOnly && materials[i].renderQueue > 2500))
                continue;
            commands.DrawRenderer(renderer, material, i, pass);
        }
    }

    private void EnsureTexture(int width, int height)
    {
        if (texture != null && texture.width == width && texture.height == height)
            return;
        ReleaseTexture();
        texture = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32)
        {
            name = "Temperature UI projection",
            hideFlags = HideFlags.HideAndDontSave,
            filterMode = FilterMode.Point,
            wrapMode = TextureWrapMode.Clamp,
            antiAliasing = 1
        };
        texture.Create();
    }

    private void ReleaseTexture()
    {
        if (texture == null)
            return;
        texture.Release();
        DestroyOwnedObject(texture);
        texture = null;
    }

    public void Dispose()
    {
        commands.Release();
        ReleaseTexture();
        if (material != null)
            DestroyOwnedObject(material);
    }

    private static void DestroyOwnedObject(UnityEngine.Object instance)
    {
        if (Application.isPlaying)
            UnityEngine.Object.Destroy(instance);
        else
            UnityEngine.Object.DestroyImmediate(instance);
    }
}

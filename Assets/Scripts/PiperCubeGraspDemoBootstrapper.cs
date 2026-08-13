using UnityEngine;

[DefaultExecutionOrder(50)]
public sealed class PiperCubeGraspDemoBootstrapper : MonoBehaviour
{
    private const string BootstrapName = "__Piper Cube Grasp Demo";
    private const string TargetCubeName = "GraspTargetCube";
    private const string LegacyTargetCubeName = "1";
    private const string LegacyDetectedObjectName = "Detected_Object";
    private const string GrabAnchorName = "Gripper_Grab_Anchor";
    private const string PlatformName = "White Platform";

    [SerializeField] private PiperArmController arm;
    [SerializeField] private Vector3 cubePosition = new(0.5f, 0.03f, 0.1f);
    [SerializeField] private Vector3 cubeScale = new(0.045f, 0.045f, 0.045f);
    [SerializeField] private float cubeMassKg = 0.04f;
    [SerializeField] private float triggerRadiusMeters = 0.1f;
    [SerializeField] private Vector3 minimumPlatformScale = new(4f, 0.1f, 4f);
    [SerializeField] private float initialGripperOpeningMeters = 0.08f;

    private static PhysicsMaterial stableDropMaterial;
    private static Mesh cubeMesh;
    private float platformTopY;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void InstallWhenSceneLoads()
    {
        if (FindAnyObjectByType<PiperCubeGraspDemoBootstrapper>() != null)
            return;

        var sceneArm = FindAnyObjectByType<PiperArmController>();
        if (sceneArm == null)
            return;

        var bootstrapObject = new GameObject(BootstrapName);
        var bootstrapper = bootstrapObject.AddComponent<PiperCubeGraspDemoBootstrapper>();
        bootstrapper.arm = sceneArm;
    }

    private void Start()
    {
        if (arm == null)
            arm = FindAnyObjectByType<PiperArmController>();
        if (arm == null)
            return;

        EnsureLargePlatform();
        GameObject targetCube = PrepareTargetCube();
        RemoveLegacySceneCubes(targetCube);
        RetargetPerceptionSimulator(targetCube.transform);
        PrepareGripperGrabber();
        arm.SetGripperOnlyMeters(initialGripperOpeningMeters, arm.GripperEffort);
        Debug.Log($"Cube grasp demo ready. Move the gripper around {targetCube.name}, press P to close and grab, press O to open and release.");
    }

    private GameObject PrepareTargetCube()
    {
        GameObject targetCube = GameObject.Find(TargetCubeName);
        if (targetCube == null)
            targetCube = FindReusableLegacyTargetCube();
        if (targetCube == null)
            targetCube = GameObject.CreatePrimitive(PrimitiveType.Cube);

        targetCube.name = TargetCubeName;
        Vector3 targetPosition = cubePosition;
        targetPosition.y = Mathf.Max(cubePosition.y, platformTopY + Mathf.Abs(cubeScale.y) * 0.5f + 0.002f);
        targetCube.transform.position = targetPosition;
        targetCube.transform.rotation = Quaternion.identity;
        targetCube.transform.localScale = cubeScale;

        var collider = targetCube.GetComponent<BoxCollider>();
        if (collider == null)
            collider = targetCube.AddComponent<BoxCollider>();
        collider.enabled = true;
        collider.isTrigger = false;
        collider.center = Vector3.zero;
        collider.size = Vector3.one;
        collider.material = StableDropMaterial();
        DisableExtraSolidColliders(targetCube, collider);

        var body = targetCube.GetComponent<Rigidbody>();
        if (body == null)
            body = targetCube.AddComponent<Rigidbody>();

        body.mass = cubeMassKg;
        body.linearDamping = 0.2f;
        body.angularDamping = 3f;
        body.maxLinearVelocity = 2f;
        body.maxAngularVelocity = 1f;
        body.solverIterations = 12;
        body.solverVelocityIterations = 8;
        body.useGravity = true;
        body.isKinematic = false;
        body.interpolation = RigidbodyInterpolation.Interpolate;
        body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;

        var grabbable = targetCube.GetComponent<PiperGrabbableObject>();
        if (grabbable == null)
            grabbable = targetCube.AddComponent<PiperGrabbableObject>();
        grabbable.ConfigureReleasePhysics(true, true);
        grabbable.ConfigureMinimumReleaseY(platformTopY);

        SafetyTemperatureZone zone = targetCube.GetComponent<SafetyTemperatureZone>();
        if (zone == null)
            zone = targetCube.AddComponent<SafetyTemperatureZone>();
        zone.Configure("Environment Object", 50f, 2f, 0.45f);

        var reset = targetCube.GetComponent<PiperGraspTargetReset>();
        if (reset == null)
            reset = targetCube.AddComponent<PiperGraspTargetReset>();
        reset.CaptureStartPose();

        return targetCube;
    }

    private void EnsureLargePlatform()
    {
        GameObject platform = GameObject.Find(PlatformName);
        if (platform == null)
        {
            platform = GameObject.CreatePrimitive(PrimitiveType.Cube);
            platform.name = PlatformName;
        }

        platform.transform.position = new Vector3(0f, -minimumPlatformScale.y * 0.5f, 0f);
        platform.transform.rotation = Quaternion.identity;
        platform.transform.localScale = minimumPlatformScale;
        platformTopY = platform.transform.position.y + Mathf.Abs(platform.transform.localScale.y) * 0.5f;

        var meshFilter = platform.GetComponent<MeshFilter>();
        if (meshFilter == null)
            meshFilter = platform.AddComponent<MeshFilter>();
        meshFilter.sharedMesh = CubeMesh();

        if (platform.GetComponent<MeshRenderer>() == null)
            platform.AddComponent<MeshRenderer>();

        var boxCollider = platform.GetComponent<BoxCollider>();
        if (boxCollider == null)
            boxCollider = platform.AddComponent<BoxCollider>();

        boxCollider.isTrigger = false;
        boxCollider.center = Vector3.zero;
        boxCollider.size = Vector3.one;

        var platformColliders = platform.GetComponents<Collider>();
        foreach (var platformCollider in platformColliders)
        {
            if (platformCollider == null)
                continue;

            platformCollider.material = StableDropMaterial();
            if (platformCollider != boxCollider)
                platformCollider.enabled = false;
        }
    }

    private static Mesh CubeMesh()
    {
        if (cubeMesh != null)
            return cubeMesh;

        GameObject temporaryCube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        temporaryCube.hideFlags = HideFlags.HideAndDontSave;
        cubeMesh = temporaryCube.GetComponent<MeshFilter>().sharedMesh;
        Destroy(temporaryCube);
        return cubeMesh;
    }

    private static PhysicsMaterial StableDropMaterial()
    {
        if (stableDropMaterial != null)
            return stableDropMaterial;

        stableDropMaterial = new PhysicsMaterial("Stable Drop Material")
        {
            dynamicFriction = 0.8f,
            staticFriction = 0.9f,
            bounciness = 0f,
            frictionCombine = PhysicsMaterialCombine.Maximum,
            bounceCombine = PhysicsMaterialCombine.Minimum
        };
        return stableDropMaterial;
    }

    private static GameObject FindReusableLegacyTargetCube()
    {
        GameObject namedLegacy = GameObject.Find(LegacyTargetCubeName);
        if (IsReusableTargetCube(namedLegacy))
            return namedLegacy;

        var zones = FindObjectsByType<SafetyTemperatureZone>();
        foreach (var zone in zones)
        {
            if (zone == null || zone.name == LegacyDetectedObjectName)
                continue;

            if (IsReusableTargetCube(zone.gameObject))
                return zone.gameObject;
        }

        return null;
    }

    private static bool IsReusableTargetCube(GameObject candidate)
    {
        if (candidate == null || candidate.name == LegacyDetectedObjectName)
            return false;

        return candidate.GetComponent<MeshFilter>() != null &&
            candidate.GetComponent<BoxCollider>() != null;
    }

    private static void RemoveLegacySceneCubes(GameObject targetCube)
    {
        RemoveLegacySceneCube(LegacyDetectedObjectName, targetCube);
        RemoveLegacySceneCube(LegacyTargetCubeName, targetCube);
    }

    private static void RemoveLegacySceneCube(string objectName, GameObject targetCube)
    {
        GameObject legacyCube = GameObject.Find(objectName);
        if (legacyCube == null || legacyCube == targetCube)
            return;

        Destroy(legacyCube);
    }

    private static void DisableExtraSolidColliders(GameObject targetCube, Collider primaryCollider)
    {
        Collider[] colliders = targetCube.GetComponentsInChildren<Collider>(true);
        for (int i = 0; i < colliders.Length; i++)
        {
            Collider candidate = colliders[i];
            if (candidate == null || candidate == primaryCollider || candidate.isTrigger)
                continue;

            candidate.enabled = false;
        }
    }

    private static void RetargetPerceptionSimulator(Transform target)
    {
        if (target == null)
            return;

        var simulators = FindObjectsByType<RealSenseCoordinateSimulator>();
        foreach (var simulator in simulators)
            simulator.SetTargetObject(target);
    }

    private void PrepareGripperGrabber()
    {
        Transform anchorParent = FindChildByName(arm.transform, "gripper_base");
        Transform link6 = FindChildByName(arm.transform, "link6");
        Transform link7 = FindChildByName(arm.transform, "link7");
        Transform link8 = FindChildByName(arm.transform, "link8");

        if (anchorParent == null)
            anchorParent = link6 != null ? link6 : arm.transform;

        Transform existingAnchor = FindChildByName(anchorParent, GrabAnchorName);
        GameObject anchorObject = existingAnchor != null ? existingAnchor.gameObject : new GameObject(GrabAnchorName);
        Transform anchor = anchorObject.transform;
        anchor.SetParent(anchorParent, false);

        if (link7 != null && link8 != null)
        {
            anchor.position = (link7.position + link8.position) * 0.5f;
            anchor.rotation = link6 != null ? link6.rotation : anchorParent.rotation;
        }
        else
        {
            anchor.localPosition = new Vector3(0f, 0.1358f, 0f);
            anchor.localRotation = Quaternion.identity;
        }

        var trigger = anchorObject.GetComponent<SphereCollider>();
        if (trigger == null)
            trigger = anchorObject.AddComponent<SphereCollider>();
        trigger.isTrigger = true;
        trigger.radius = triggerRadiusMeters;

        var grabber = anchorObject.GetComponent<PiperGripperGrabber>();
        if (grabber == null)
            grabber = anchorObject.AddComponent<PiperGripperGrabber>();

        ArticulationBody graspBody = anchorParent.GetComponent<ArticulationBody>();
        if (graspBody == null)
            graspBody = arm.GetComponentInChildren<ArticulationBody>();

        grabber.Configure(arm, anchor, trigger, link7, link8, graspBody);
    }

    private static Transform FindChildByName(Transform root, string childName)
    {
        if (root == null)
            return null;
        if (root.name == childName)
            return root;

        for (int i = 0; i < root.childCount; i++)
        {
            Transform found = FindChildByName(root.GetChild(i), childName);
            if (found != null)
                return found;
        }

        return null;
    }

}

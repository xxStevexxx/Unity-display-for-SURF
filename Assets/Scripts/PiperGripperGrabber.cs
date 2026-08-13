using System.Collections.Generic;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class PiperGripperGrabber : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private PiperArmController arm;
    [SerializeField] private Transform gripAnchor;
    [SerializeField] private Transform leftFinger;
    [SerializeField] private Transform rightFinger;
    [SerializeField] private Collider detectionTrigger;

    [Header("Grasp thresholds")]
    [SerializeField] private float releaseOpeningMeters = 0.06f;
    [SerializeField] private float maxGrabDistanceMeters = 0.1f;
    [SerializeField] private float minimumObjectWidthMeters = 0.012f;
    [SerializeField] private float maximumObjectWidthMeters = 0.085f;
    [SerializeField] private float contactToleranceMeters = 0.006f;
    [SerializeField] private float fingerContactToleranceMeters = 0.006f;
    [SerializeField] private float releaseClearanceMeters = 0.01f;
    [SerializeField] private float maxReleaseOpeningMeters = 0.079f;
    [SerializeField] private float centerToleranceMeters = 0.025f;
    [SerializeField] private float depthToleranceMeters = 0.055f;
    [SerializeField] private float heightToleranceMeters = 0.05f;
    [SerializeField] private float stopClearanceMeters = 0.0015f;
    [SerializeField] private float closingSpeedThresholdMeters = 0.00005f;
    [SerializeField] private float releaseDownwardSpeedMetersPerSecond = 0.02f;
    [SerializeField] private float releaseCollisionGraceSeconds = 0.6f;
    [SerializeField] private float regrabCooldownSeconds = 0.15f;

    [Header("Physical finger contact")]
    [SerializeField] private float surfaceProbeRadiusMeters = 0.0025f;
    [SerializeField] private float surfaceProbeInsetMeters = 0.0005f;
    [SerializeField] private int probesPerAxis = 3;

    private readonly HashSet<PiperGrabbableObject> candidates = new();
    private readonly HashSet<PiperGrabbableObject> openedAroundCandidates = new();
    private readonly HashSet<PiperGrabbableObject> closingAroundCandidates = new();

    private struct GraspGeometry
    {
        public float DistanceMeters;
        public float CenterOffsetMeters;
        public float DepthOffsetMeters;
        public float HeightOffsetMeters;
        public float RequiredOpeningMeters;
    }

    private PiperGrabbableObject heldObject;
    private float previousGripperOpeningMeters = 0.08f;
    private float heldObjectWidthMeters;
    private Collider[] releaseIgnoredColliders;
    private Collider[] leftFingerContactColliders;
    private Collider[] rightFingerContactColliders;
    private readonly List<Transform> leftSurfaceProbes = new();
    private readonly List<Transform> rightSurfaceProbes = new();
    private ArticulationBody graspBody;
    private bool surfaceProbesReady;
    private float nextGrabAllowedTime;

    public bool HasHeldObject => heldObject != null;

    private void Awake()
    {
        if (arm == null)
            arm = GetComponentInParent<PiperArmController>();
        if (gripAnchor == null)
            gripAnchor = transform;
        if (detectionTrigger == null)
            detectionTrigger = EnsureDefaultTrigger();

        RefreshReleaseIgnoredColliders();
        RefreshFingerPhysicsColliders();
        RefreshFingerContactColliders();
        RefreshSurfaceProbes();
        previousGripperOpeningMeters = CurrentGripperOpeningMeters();
    }

    private void FixedUpdate()
    {
        float currentOpeningMeters = CurrentGripperOpeningMeters();

        if (heldObject != null)
        {
            if (ShouldRelease(currentOpeningMeters))
                ReleaseHeldObject();
            previousGripperOpeningMeters = currentOpeningMeters;
            return;
        }

        if (arm != null && arm.IsEnabled && Time.time >= nextGrabAllowedTime)
            TryGrabNearestCandidate(currentOpeningMeters, previousGripperOpeningMeters);

        previousGripperOpeningMeters = currentOpeningMeters;
    }

    private void OnTriggerEnter(Collider other)
    {
        var grabbable = other.GetComponentInParent<PiperGrabbableObject>();
        if (grabbable != null && !grabbable.IsHeld)
            candidates.Add(grabbable);
    }

    private void OnTriggerExit(Collider other)
    {
        var grabbable = other.GetComponentInParent<PiperGrabbableObject>();
        if (grabbable != null && grabbable != heldObject)
        {
            candidates.Remove(grabbable);
            openedAroundCandidates.Remove(grabbable);
            closingAroundCandidates.Remove(grabbable);
        }
    }

    public void Configure(
        PiperArmController sourceArm,
        Transform sourceAnchor,
        Collider sourceTrigger,
        Transform sourceLeftFinger = null,
        Transform sourceRightFinger = null,
        ArticulationBody sourceGraspBody = null)
    {
        arm = sourceArm;
        gripAnchor = sourceAnchor != null ? sourceAnchor : transform;
        leftFinger = sourceLeftFinger;
        rightFinger = sourceRightFinger;
        graspBody = sourceGraspBody != null
            ? sourceGraspBody
            : gripAnchor.GetComponentInParent<ArticulationBody>();
        detectionTrigger = sourceTrigger != null ? sourceTrigger : EnsureDefaultTrigger();
        RefreshReleaseIgnoredColliders();
        RefreshFingerPhysicsColliders();
        RefreshFingerContactColliders();
        RefreshSurfaceProbes();
        previousGripperOpeningMeters = CurrentGripperOpeningMeters();
    }

    private void TryGrabNearestCandidate(float currentOpeningMeters, float previousOpeningMeters)
    {
        RefreshNearbyCandidates();

        PiperGrabbableObject bestCandidate = null;
        float bestScore = float.MaxValue;
        float bestWidthMeters = 0f;

        candidates.RemoveWhere(candidate => candidate == null || candidate.IsHeld || !candidate.gameObject.activeInHierarchy);
        openedAroundCandidates.RemoveWhere(candidate => candidate == null || candidate.IsHeld || !candidate.gameObject.activeInHierarchy);
        closingAroundCandidates.RemoveWhere(candidate => candidate == null || candidate.IsHeld || !candidate.gameObject.activeInHierarchy);

        foreach (var candidate in candidates)
        {
            if (!TryEvaluateCandidate(candidate, currentOpeningMeters, previousOpeningMeters, out float score, out float objectWidthMeters))
                continue;

            if (score >= bestScore)
                continue;

            bestCandidate = candidate;
            bestScore = score;
            bestWidthMeters = objectWidthMeters;
        }

        if (bestCandidate == null)
            return;

        heldObject = bestCandidate;
        heldObjectWidthMeters = bestWidthMeters;
        bool attached = graspBody != null
            ? heldObject.AttachWithPhysics(gripAnchor, graspBody)
            : false;
        if (!attached)
        {
            heldObject = null;
            return;
        }

        var reset = heldObject.GetComponent<PiperGraspTargetReset>();
        if (reset != null)
            reset.MarkHeld();
    }

    private bool TryEvaluateCandidate(
        PiperGrabbableObject candidate,
        float currentOpeningMeters,
        float previousOpeningMeters,
        out float score,
        out float objectWidthMeters)
    {
        score = float.MaxValue;
        objectWidthMeters = 0f;
        if (candidate == null || gripAnchor == null)
            return false;

        Vector3 widthAxis = FingerWidthAxis();
        objectWidthMeters = candidate.GetWidthAlongAxis(widthAxis);
        if (objectWidthMeters < minimumObjectWidthMeters || objectWidthMeters > maximumObjectWidthMeters)
            return false;

        if (!TryGetGraspGeometry(candidate, objectWidthMeters, out GraspGeometry geometry))
            return false;

        float openEnoughThreshold = geometry.RequiredOpeningMeters - contactToleranceMeters * 0.5f;
        if (currentOpeningMeters >= openEnoughThreshold || previousOpeningMeters >= openEnoughThreshold)
            openedAroundCandidates.Add(candidate);

        if (!openedAroundCandidates.Contains(candidate))
            return false;

        bool gripperIsOpening = currentOpeningMeters > previousOpeningMeters + closingSpeedThresholdMeters;
        if (gripperIsOpening)
        {
            closingAroundCandidates.Remove(candidate);
            return false;
        }

        bool gripperIsClosing = previousOpeningMeters > currentOpeningMeters + closingSpeedThresholdMeters;
        if (gripperIsClosing)
            closingAroundCandidates.Add(candidate);

        if (!closingAroundCandidates.Contains(candidate))
            return false;

        if (gripperIsOpening)
            return false;

        if (!HasTwoFingerContact(candidate, out float contactDistanceMeters))
            return false;

        float openingError = Mathf.Abs(currentOpeningMeters - geometry.RequiredOpeningMeters);
        score = geometry.DistanceMeters +
            geometry.CenterOffsetMeters * 2f +
            geometry.DepthOffsetMeters +
            geometry.HeightOffsetMeters +
            openingError * 4f +
            contactDistanceMeters * 3f;
        return true;
    }

    private bool ShouldRelease(float currentOpeningMeters)
    {
        if (arm == null || !arm.IsEnabled)
            return true;

        float objectReleaseOpening = heldObjectWidthMeters > 0f
            ? Mathf.Min(heldObjectWidthMeters + releaseClearanceMeters, maxReleaseOpeningMeters)
            : releaseOpeningMeters;
        return currentOpeningMeters >= objectReleaseOpening;
    }

    private void ReleaseHeldObject()
    {
        RefreshReleaseIgnoredColliders();
        Vector3 releaseVelocity = Vector3.down * Mathf.Max(0f, releaseDownwardSpeedMetersPerSecond);

        PiperGrabbableObject releasedObject = heldObject;
        heldObject = null;

        releasedObject.Release(releaseVelocity, releaseIgnoredColliders, releaseCollisionGraceSeconds);
        ResetCandidateState(releasedObject);
        candidates.Add(releasedObject);
        nextGrabAllowedTime = Time.time + regrabCooldownSeconds;
        heldObjectWidthMeters = 0f;
    }

    private float CurrentGripperOpeningMeters()
    {
        if (arm == null)
            return releaseOpeningMeters;

        if (arm.LastCommand != null)
            return (float)arm.LastCommand.GripperMeters;
        if (arm.Feedback != null)
            return (float)arm.Feedback.GripperMeters;

        return releaseOpeningMeters;
    }

    public double ConstrainRequestedOpening(double requestedOpeningMeters)
    {
        float requested = Mathf.Clamp((float)requestedOpeningMeters, 0f, 0.08f);
        float current = CurrentGripperOpeningMeters();
        if (requested >= current - closingSpeedThresholdMeters)
            return requested;

        if (heldObject != null && heldObjectWidthMeters > 0f)
        {
            float heldStopOpening = Mathf.Max(heldObjectWidthMeters + stopClearanceMeters, heldObjectWidthMeters);
            return Mathf.Clamp(Mathf.Max(requested, heldStopOpening), 0f, 0.08f);
        }

        return requested;
    }

    private Vector3 FingerWidthAxis()
    {
        if (leftFinger != null && rightFinger != null)
        {
            Vector3 betweenFingers = rightFinger.position - leftFinger.position;
            if (betweenFingers.sqrMagnitude > 0.000001f)
                return betweenFingers.normalized;
        }

        return gripAnchor != null ? gripAnchor.right.normalized : transform.right.normalized;
    }

    private Vector3 FingerDepthAxis()
    {
        if (gripAnchor != null)
            return gripAnchor.up.normalized;

        return transform.up.normalized;
    }

    private Vector3 FingerHeightAxis()
    {
        Vector3 widthAxis = FingerWidthAxis();
        Vector3 depthAxis = FingerDepthAxis();
        Vector3 heightAxis = Vector3.Cross(widthAxis, depthAxis);
        if (heightAxis.sqrMagnitude < 0.000001f)
            return gripAnchor != null ? gripAnchor.forward.normalized : transform.forward.normalized;

        return heightAxis.normalized;
    }

    private bool TryGetGraspGeometry(PiperGrabbableObject candidate, float objectWidthMeters, out GraspGeometry geometry)
    {
        geometry = default;
        if (candidate == null || gripAnchor == null)
            return false;

        Vector3 toObject = candidate.GrabPoint - GripperCenter();
        Vector3 widthAxis = FingerWidthAxis();
        Vector3 depthAxis = FingerDepthAxis();
        Vector3 heightAxis = FingerHeightAxis();

        geometry.DistanceMeters = toObject.magnitude;
        geometry.CenterOffsetMeters = Mathf.Abs(Vector3.Dot(toObject, widthAxis));
        geometry.DepthOffsetMeters = Mathf.Abs(Vector3.Dot(toObject, depthAxis));
        geometry.HeightOffsetMeters = Mathf.Abs(Vector3.Dot(toObject, heightAxis));
        geometry.RequiredOpeningMeters = objectWidthMeters + geometry.CenterOffsetMeters * 2f + stopClearanceMeters;

        if (geometry.DistanceMeters > maxGrabDistanceMeters)
            return false;
        if (geometry.CenterOffsetMeters > centerToleranceMeters)
            return false;
        if (geometry.DepthOffsetMeters > depthToleranceMeters)
            return false;
        if (geometry.HeightOffsetMeters > heightToleranceMeters)
            return false;

        return true;
    }

    private Vector3 GripperCenter()
    {
        if (leftFinger != null && rightFinger != null)
            return (leftFinger.position + rightFinger.position) * 0.5f;

        return gripAnchor != null ? gripAnchor.position : transform.position;
    }

    private void RefreshNearbyCandidates()
    {
        if (gripAnchor == null)
            return;

        var grabbables = FindObjectsByType<PiperGrabbableObject>();
        float maxSqrDistance = maxGrabDistanceMeters * maxGrabDistanceMeters;
        foreach (var grabbable in grabbables)
        {
            if (grabbable == null || grabbable.IsHeld || !grabbable.gameObject.activeInHierarchy)
                continue;
            if ((grabbable.GrabPoint - GripperCenter()).sqrMagnitude > maxSqrDistance)
                continue;

            candidates.Add(grabbable);
        }
    }

    private void RefreshReleaseIgnoredColliders()
    {
        releaseIgnoredColliders = arm != null
            ? arm.GetComponentsInChildren<Collider>(true)
            : GetComponentsInParent<Collider>(true);
    }

    private void RefreshFingerContactColliders()
    {
        leftFingerContactColliders = CollectFingerContactColliders(leftFinger);
        rightFingerContactColliders = CollectFingerContactColliders(rightFinger);
    }

    private void RefreshFingerPhysicsColliders()
    {
        EnsureSolidFingerCollider(leftFinger);
        EnsureSolidFingerCollider(rightFinger);
    }

    private void EnsureSolidFingerCollider(Transform finger)
    {
        if (finger == null)
            return;

        var meshFilters = finger.GetComponentsInChildren<MeshFilter>(true);
        for (int i = 0; i < meshFilters.Length; i++)
        {
            MeshFilter meshFilter = meshFilters[i];
            if (meshFilter == null || meshFilter.sharedMesh == null)
                continue;

            Collider[] existing = meshFilter.GetComponents<Collider>();
            BoxCollider solidBox = null;
            for (int j = 0; j < existing.Length; j++)
            {
                if (existing[j] is BoxCollider box && !box.isTrigger)
                {
                    solidBox = box;
                    break;
                }
            }

            if (solidBox == null)
                solidBox = meshFilter.gameObject.AddComponent<BoxCollider>();

            solidBox.isTrigger = false;
            solidBox.enabled = true;
            solidBox.center = meshFilter.sharedMesh.bounds.center;
            solidBox.size = meshFilter.sharedMesh.bounds.size;
            solidBox.material = null;
        }
    }

    private void RefreshSurfaceProbes()
    {
        DestroyGeneratedProbes(leftSurfaceProbes);
        DestroyGeneratedProbes(rightSurfaceProbes);
        surfaceProbesReady = false;

        if (leftFinger == null || rightFinger == null)
            return;

        BuildSurfaceProbes(leftFinger, rightFinger, leftSurfaceProbes, "Left");
        BuildSurfaceProbes(rightFinger, leftFinger, rightSurfaceProbes, "Right");
        surfaceProbesReady = leftSurfaceProbes.Count > 0 && rightSurfaceProbes.Count > 0;
    }

    private void DestroyGeneratedProbes(List<Transform> probes)
    {
        for (int i = 0; i < probes.Count; i++)
        {
            if (probes[i] != null)
                Destroy(probes[i].gameObject);
        }

        probes.Clear();
    }

    private void BuildSurfaceProbes(
        Transform finger,
        Transform oppositeFinger,
        List<Transform> probes,
        string sideName)
    {
        MeshFilter meshFilter = null;
        var meshFilters = finger.GetComponentsInChildren<MeshFilter>(true);
        for (int i = 0; i < meshFilters.Length; i++)
        {
            if (meshFilters[i] != null && meshFilters[i].sharedMesh != null)
            {
                meshFilter = meshFilters[i];
                break;
            }
        }

        if (meshFilter == null || oppositeFinger == null)
            return;

        Bounds bounds = meshFilter.sharedMesh.bounds;
        Vector3 toOppositeLocal = meshFilter.transform.InverseTransformPoint(oppositeFinger.position) - bounds.center;
        Vector3[] axes = { Vector3.right, Vector3.up, Vector3.forward };
        int faceAxis = 0;
        float largest = Mathf.Abs(toOppositeLocal.x);
        if (Mathf.Abs(toOppositeLocal.y) > largest)
        {
            faceAxis = 1;
            largest = Mathf.Abs(toOppositeLocal.y);
        }
        if (Mathf.Abs(toOppositeLocal.z) > largest)
            faceAxis = 2;

        Vector3 faceDirection = axes[faceAxis];
        float faceSign = Vector3.Dot(toOppositeLocal, faceDirection) >= 0f ? 1f : -1f;
        Vector3 extents = bounds.extents;
        float faceInset = Mathf.Min(surfaceProbeInsetMeters, extents[faceAxis] * 0.5f);
        Vector3 faceCenter = bounds.center + faceDirection * faceSign * (extents[faceAxis] - faceInset);

        int gridSize = Mathf.Clamp(probesPerAxis, 2, 5);
        int firstAxis = (faceAxis + 1) % 3;
        int secondAxis = (faceAxis + 2) % 3;
        Vector3 firstDirection = axes[firstAxis];
        Vector3 secondDirection = axes[secondAxis];
        float firstExtent = extents[firstAxis] * 0.8f;
        float secondExtent = extents[secondAxis] * 0.8f;

        for (int row = 0; row < gridSize; row++)
        {
            float firstT = gridSize == 1 ? 0.5f : (float)row / (gridSize - 1);
            float firstOffset = Mathf.Lerp(-firstExtent, firstExtent, firstT);
            for (int column = 0; column < gridSize; column++)
            {
                float secondT = gridSize == 1 ? 0.5f : (float)column / (gridSize - 1);
                float secondOffset = Mathf.Lerp(-secondExtent, secondExtent, secondT);
                var probeObject = new GameObject($"{sideName}_InnerSurfaceProbe_{row}_{column}");
                probeObject.transform.SetParent(meshFilter.transform, false);
                probeObject.transform.localPosition = faceCenter + firstDirection * firstOffset + secondDirection * secondOffset;
                probeObject.layer = meshFilter.gameObject.layer;
                probeObject.hideFlags = HideFlags.DontSave;

                var probeCollider = probeObject.AddComponent<SphereCollider>();
                probeCollider.isTrigger = true;
                probeCollider.radius = Mathf.Max(0.0005f, surfaceProbeRadiusMeters);
                probes.Add(probeObject.transform);
            }
        }
    }

    private Collider[] CollectFingerContactColliders(Transform finger)
    {
        if (finger == null)
            return System.Array.Empty<Collider>();

        var colliders = new List<Collider>();
        var meshFilters = finger.GetComponentsInChildren<MeshFilter>(true);
        for (int i = 0; i < meshFilters.Length; i++)
        {
            MeshFilter meshFilter = meshFilters[i];
            if (meshFilter == null || meshFilter.sharedMesh == null)
                continue;

            var box = FindReusableContactBox(meshFilter.gameObject);
            if (box == null)
                box = meshFilter.gameObject.AddComponent<BoxCollider>();

            box.isTrigger = true;
            box.enabled = true;
            box.center = meshFilter.sharedMesh.bounds.center;
            box.size = meshFilter.sharedMesh.bounds.size;
            if (IsUsableFingerCollider(box))
                colliders.Add(box);
        }

        if (colliders.Count > 0)
            return colliders.ToArray();

        var existingColliders = finger.GetComponentsInChildren<Collider>(true);
        for (int i = 0; i < existingColliders.Length; i++)
        {
            Collider candidate = existingColliders[i];
            if (IsUsableFingerCollider(candidate))
                colliders.Add(candidate);
        }

        return colliders.ToArray();
    }

    private BoxCollider FindReusableContactBox(GameObject owner)
    {
        if (owner == null)
            return null;

        var boxes = owner.GetComponents<BoxCollider>();
        for (int i = 0; i < boxes.Length; i++)
        {
            BoxCollider box = boxes[i];
            if (box != null && box.isTrigger && box != detectionTrigger)
                return box;
        }

        return null;
    }

    private bool IsUsableFingerCollider(Collider candidate)
    {
        if (candidate == null || !candidate.enabled || candidate == detectionTrigger)
            return false;

        Bounds bounds = candidate.bounds;
        return bounds.size.sqrMagnitude > 0.000001f;
    }

    private bool HasTwoFingerContact(PiperGrabbableObject candidate, out float contactDistanceMeters)
    {
        bool leftContact = false;
        bool rightContact = false;
        GetFingerContactState(candidate, out leftContact, out rightContact, out contactDistanceMeters);
        return leftContact && rightContact;
    }

    private void GetFingerContactState(
        PiperGrabbableObject candidate,
        out bool leftContact,
        out bool rightContact,
        out float contactDistanceMeters)
    {
        leftContact = false;
        rightContact = false;
        contactDistanceMeters = float.MaxValue;
        if (candidate == null)
            return;

        if (leftFingerContactColliders == null || leftFingerContactColliders.Length == 0 ||
            rightFingerContactColliders == null || rightFingerContactColliders.Length == 0)
        {
            RefreshFingerContactColliders();
        }

        Collider[] objectColliders = candidate.GetContactColliders();
        float leftDistance = MinimumColliderDistance(objectColliders, leftFingerContactColliders);
        float rightDistance = MinimumColliderDistance(objectColliders, rightFingerContactColliders);

        leftContact = leftDistance <= fingerContactToleranceMeters;
        rightContact = rightDistance <= fingerContactToleranceMeters;

        if (surfaceProbesReady)
        {
            leftContact |= HasSurfaceProbeContact(candidate, leftSurfaceProbes);
            rightContact |= HasSurfaceProbeContact(candidate, rightSurfaceProbes);
        }

        contactDistanceMeters = Mathf.Min(leftDistance, fingerContactToleranceMeters) +
            Mathf.Min(rightDistance, fingerContactToleranceMeters);
    }

    private bool HasSurfaceProbeContact(PiperGrabbableObject candidate, List<Transform> probes)
    {
        if (candidate == null || probes == null || probes.Count == 0)
            return false;

        for (int i = 0; i < probes.Count; i++)
        {
            Transform probe = probes[i];
            if (probe == null)
                continue;

            Collider[] overlaps = Physics.OverlapSphere(
                probe.position,
                surfaceProbeRadiusMeters,
                Physics.AllLayers,
                QueryTriggerInteraction.Ignore);

            for (int j = 0; j < overlaps.Length; j++)
            {
                PiperGrabbableObject hit = overlaps[j].GetComponentInParent<PiperGrabbableObject>();
                if (hit == candidate)
                    return true;
            }
        }

        return false;
    }

    private float MinimumColliderDistance(Collider[] objectColliders, Collider[] fingerColliders)
    {
        if (objectColliders == null || fingerColliders == null ||
            objectColliders.Length == 0 || fingerColliders.Length == 0)
            return float.MaxValue;

        float bestDistance = float.MaxValue;
        for (int i = 0; i < objectColliders.Length; i++)
        {
            Collider objectCollider = objectColliders[i];
            if (objectCollider == null || !objectCollider.enabled || objectCollider.isTrigger)
                continue;

            for (int j = 0; j < fingerColliders.Length; j++)
            {
                Collider fingerCollider = fingerColliders[j];
                if (!IsUsableFingerCollider(fingerCollider))
                    continue;

                float distance = ApproximateColliderDistance(objectCollider, fingerCollider);
                if (distance < bestDistance)
                    bestDistance = distance;
            }
        }

        return bestDistance;
    }

    private static float ApproximateColliderDistance(Collider a, Collider b)
    {
        Vector3 penetrationDirection;
        float penetrationDistance;
        if (Physics.ComputePenetration(
            a,
            a.transform.position,
            a.transform.rotation,
            b,
            b.transform.position,
            b.transform.rotation,
            out penetrationDirection,
            out penetrationDistance))
        {
            return 0f;
        }

        Vector3 aPoint = a.ClosestPoint(b.bounds.center);
        Vector3 bPoint = b.ClosestPoint(aPoint);
        float distance = Vector3.Distance(aPoint, bPoint);

        Vector3 bPointFromCenter = b.ClosestPoint(a.bounds.center);
        Vector3 aPointFromCenter = a.ClosestPoint(bPointFromCenter);
        return Mathf.Min(distance, Vector3.Distance(aPointFromCenter, bPointFromCenter));
    }

    private Collider EnsureDefaultTrigger()
    {
        var sphere = GetComponent<SphereCollider>();
        if (sphere == null)
            sphere = gameObject.AddComponent<SphereCollider>();

        sphere.isTrigger = true;
        sphere.radius = maxGrabDistanceMeters;
        return sphere;
    }

    private void ResetCandidateState(PiperGrabbableObject candidate)
    {
        candidates.Remove(candidate);
        openedAroundCandidates.Remove(candidate);
        closingAroundCandidates.Remove(candidate);
    }
}

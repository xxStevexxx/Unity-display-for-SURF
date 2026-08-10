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
    [SerializeField] private float closingSpeedThresholdMeters = 0.00005f;
    [SerializeField] private float releaseDownwardSpeedMetersPerSecond = 0.02f;
    [SerializeField] private float releaseCollisionGraceSeconds = 0.6f;
    [SerializeField] private float regrabCooldownSeconds = 0.15f;

    private readonly HashSet<PiperGrabbableObject> candidates = new();
    private readonly HashSet<PiperGrabbableObject> openedAroundCandidates = new();
    private readonly HashSet<PiperGrabbableObject> closingAroundCandidates = new();
    private PiperGrabbableObject heldObject;
    private float previousGripperOpeningMeters = 0.08f;
    private float heldObjectWidthMeters;
    private Collider[] releaseIgnoredColliders;
    private Collider[] leftFingerContactColliders;
    private Collider[] rightFingerContactColliders;
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
        RefreshFingerContactColliders();
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
        Transform sourceRightFinger = null)
    {
        arm = sourceArm;
        gripAnchor = sourceAnchor != null ? sourceAnchor : transform;
        leftFinger = sourceLeftFinger;
        rightFinger = sourceRightFinger;
        detectionTrigger = sourceTrigger != null ? sourceTrigger : EnsureDefaultTrigger();
        RefreshReleaseIgnoredColliders();
        RefreshFingerContactColliders();
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
        heldObject.AttachTo(gripAnchor);
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

        Vector3 toObject = candidate.GrabPoint - gripAnchor.position;
        float distanceMeters = toObject.magnitude;
        if (distanceMeters > maxGrabDistanceMeters)
            return false;

        float centerOffsetMeters = Mathf.Abs(Vector3.Dot(toObject, widthAxis));
        if (centerOffsetMeters > centerToleranceMeters)
            return false;

        float openEnoughThreshold = objectWidthMeters - contactToleranceMeters * 0.5f;
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

        bool hasReachedObjectSurface = currentOpeningMeters <= objectWidthMeters + contactToleranceMeters;
        if (gripperIsOpening || !hasReachedObjectSurface)
            return false;

        if (!HasTwoFingerContact(candidate, out float contactDistanceMeters))
            return false;

        float openingError = Mathf.Abs(currentOpeningMeters - objectWidthMeters);
        score = distanceMeters + centerOffsetMeters * 2f + openingError * 4f + contactDistanceMeters * 3f;
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
            if ((grabbable.GrabPoint - gripAnchor.position).sqrMagnitude > maxSqrDistance)
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
        contactDistanceMeters = float.MaxValue;
        if (candidate == null)
            return false;

        if (leftFingerContactColliders == null || leftFingerContactColliders.Length == 0 ||
            rightFingerContactColliders == null || rightFingerContactColliders.Length == 0)
        {
            RefreshFingerContactColliders();
        }

        Collider[] objectColliders = candidate.GetContactColliders();
        float leftDistance = MinimumColliderDistance(objectColliders, leftFingerContactColliders);
        float rightDistance = MinimumColliderDistance(objectColliders, rightFingerContactColliders);

        if (leftDistance > fingerContactToleranceMeters || rightDistance > fingerContactToleranceMeters)
            return false;

        contactDistanceMeters = leftDistance + rightDistance;
        return true;
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

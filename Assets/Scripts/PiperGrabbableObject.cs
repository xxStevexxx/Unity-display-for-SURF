using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Collider))]
[RequireComponent(typeof(Rigidbody))]
public sealed class PiperGrabbableObject : MonoBehaviour
{
    private struct IgnoredCollisionPair
    {
        public Collider Self;
        public Collider Other;
    }

    private Rigidbody body;
    private Collider[] cachedColliders;
    private Renderer[] cachedRenderers;
    private readonly List<IgnoredCollisionPair> ignoredCollisionPairs = new();
    private Transform originalParent;
    private bool originalUseGravity;
    private bool originalIsKinematic;
    private RigidbodyInterpolation originalInterpolation;
    private CollisionDetectionMode originalCollisionMode;
    private bool releaseWithPhysics = true;
    private bool useGravityOnRelease = true;
    private bool hasMinimumReleaseY;
    private float minimumReleaseY;
    private Vector3 heldLocalPosition;
    private Quaternion heldLocalRotation;
    private Transform currentAnchor;
    private ConfigurableJoint graspJoint;
    private Coroutine collisionRestoreRoutine;

    public bool IsHeld => currentAnchor != null || graspJoint != null;
    public Vector3 GrabPoint => body != null ? body.worldCenterOfMass : transform.position;

    private void Awake()
    {
        body = GetComponent<Rigidbody>();
        cachedColliders = GetComponentsInChildren<Collider>();
        cachedRenderers = GetComponentsInChildren<Renderer>();
    }

    public void ConfigureReleasePhysics(bool releaseAsDynamicBody, bool useGravity)
    {
        releaseWithPhysics = releaseAsDynamicBody;
        useGravityOnRelease = useGravity;
    }

    public void ConfigureMinimumReleaseY(float worldY)
    {
        hasMinimumReleaseY = true;
        minimumReleaseY = worldY;
    }

    public Bounds GetWorldBounds()
    {
        bool hasBounds = false;
        Bounds bounds = new Bounds(transform.position, Vector3.zero);

        if (cachedColliders == null || cachedColliders.Length == 0)
            cachedColliders = GetComponentsInChildren<Collider>();

        for (int i = 0; i < cachedColliders.Length; i++)
        {
            Collider candidate = cachedColliders[i];
            if (candidate == null || !candidate.enabled || candidate.isTrigger)
                continue;

            if (!hasBounds)
            {
                bounds = candidate.bounds;
                hasBounds = true;
            }
            else
            {
                bounds.Encapsulate(candidate.bounds);
            }
        }

        if (!hasBounds)
        {
            if (cachedRenderers == null || cachedRenderers.Length == 0)
                cachedRenderers = GetComponentsInChildren<Renderer>();

            for (int i = 0; i < cachedRenderers.Length; i++)
            {
                Renderer candidate = cachedRenderers[i];
                if (candidate == null || !candidate.enabled)
                    continue;

                if (!hasBounds)
                {
                    bounds = candidate.bounds;
                    hasBounds = true;
                }
                else
                {
                    bounds.Encapsulate(candidate.bounds);
                }
            }
        }

        if (!hasBounds)
            bounds = new Bounds(transform.position, transform.lossyScale);

        return bounds;
    }

    public float GetWidthAlongAxis(Vector3 worldAxis)
    {
        if (worldAxis.sqrMagnitude < 0.0001f)
            return 0f;

        Vector3 axis = worldAxis.normalized;
        Bounds bounds = GetWorldBounds();
        Vector3 extents = bounds.extents;
        float radius =
            Mathf.Abs(axis.x) * extents.x +
            Mathf.Abs(axis.y) * extents.y +
            Mathf.Abs(axis.z) * extents.z;
        return radius * 2f;
    }

    public Collider[] GetContactColliders()
    {
        RefreshCachedColliders();
        return cachedColliders;
    }

    private void LateUpdate()
    {
        if (currentAnchor == null)
            return;

        transform.localPosition = heldLocalPosition;
        transform.localRotation = heldLocalRotation;
    }

    public void AttachTo(Transform anchor)
    {
        if (anchor == null)
            return;

        RestoreIgnoredCollisions();

        if (body == null)
            body = GetComponent<Rigidbody>();

        originalParent = transform.parent;
        originalUseGravity = body.useGravity;
        originalIsKinematic = body.isKinematic;
        originalInterpolation = body.interpolation;
        originalCollisionMode = body.collisionDetectionMode;

        if (!body.isKinematic)
        {
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
        }
        body.useGravity = false;
        body.isKinematic = true;
        body.interpolation = RigidbodyInterpolation.Interpolate;
        body.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;

        transform.SetParent(anchor, true);
        heldLocalPosition = transform.localPosition;
        heldLocalRotation = transform.localRotation;
        currentAnchor = anchor;
    }

    public bool AttachWithPhysics(Transform anchor, ArticulationBody connectedBody)
    {
        if (anchor == null || connectedBody == null)
            return false;

        RestoreIgnoredCollisions();

        if (body == null)
            body = GetComponent<Rigidbody>();

        if (body == null)
            return false;

        if (graspJoint != null)
            Destroy(graspJoint);

        originalParent = transform.parent;
        originalUseGravity = body.useGravity;
        originalIsKinematic = body.isKinematic;
        originalInterpolation = body.interpolation;
        originalCollisionMode = body.collisionDetectionMode;

        Vector3 graspWorldPosition = anchor.position;
        Quaternion graspWorldRotation = anchor.rotation;

        body.useGravity = false;
        body.isKinematic = false;
        body.interpolation = RigidbodyInterpolation.Interpolate;
        body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        body.linearVelocity = Vector3.zero;
        body.angularVelocity = Vector3.zero;

        var joint = gameObject.AddComponent<ConfigurableJoint>();
        joint.connectedBody = null;
        joint.connectedArticulationBody = connectedBody;
        joint.autoConfigureConnectedAnchor = false;
        joint.anchor = transform.InverseTransformPoint(graspWorldPosition);
        joint.connectedAnchor = connectedBody.transform.InverseTransformPoint(graspWorldPosition);
        joint.axis = transform.InverseTransformDirection(graspWorldRotation * Vector3.right).normalized;
        joint.secondaryAxis = transform.InverseTransformDirection(graspWorldRotation * Vector3.up).normalized;
        joint.xMotion = ConfigurableJointMotion.Locked;
        joint.yMotion = ConfigurableJointMotion.Locked;
        joint.zMotion = ConfigurableJointMotion.Locked;
        joint.angularXMotion = ConfigurableJointMotion.Locked;
        joint.angularYMotion = ConfigurableJointMotion.Locked;
        joint.angularZMotion = ConfigurableJointMotion.Locked;
        joint.enableCollision = true;
        joint.projectionMode = JointProjectionMode.PositionAndRotation;
        joint.breakForce = Mathf.Infinity;
        joint.breakTorque = Mathf.Infinity;
        graspJoint = joint;
        body.WakeUp();
        return true;
    }

    public void Release(Vector3 releaseVelocity, Collider[] temporaryIgnoredColliders = null, float collisionGraceSeconds = 0f)
    {
        if (body == null || (!IsHeld && graspJoint == null))
            return;

        if (currentAnchor != null)
            transform.SetParent(originalParent, true);
        currentAnchor = null;

        if (graspJoint != null)
        {
            Destroy(graspJoint);
            graspJoint = null;
        }

        ApplyMinimumReleaseHeight();
        BeginTemporaryCollisionIgnore(temporaryIgnoredColliders, collisionGraceSeconds);

        body.isKinematic = releaseWithPhysics ? false : originalIsKinematic;
        body.useGravity = releaseWithPhysics ? useGravityOnRelease : originalUseGravity;
        body.interpolation = originalInterpolation;
        body.collisionDetectionMode = originalCollisionMode;

        if (!body.isKinematic)
        {
            body.linearVelocity = releaseVelocity;
            body.angularVelocity = Vector3.zero;
            body.WakeUp();
        }
    }

    private void ApplyMinimumReleaseHeight()
    {
        if (!hasMinimumReleaseY)
            return;

        Bounds bounds = GetWorldBounds();
        float minimumAllowedY = minimumReleaseY + 0.001f;
        if (bounds.min.y >= minimumAllowedY)
            return;

        transform.position += Vector3.up * (minimumAllowedY - bounds.min.y);
    }

    private void BeginTemporaryCollisionIgnore(Collider[] otherColliders, float minimumSeconds)
    {
        RestoreIgnoredCollisions();
        if (otherColliders == null || otherColliders.Length == 0 || minimumSeconds <= 0f)
            return;

        RefreshCachedColliders();
        for (int i = 0; i < cachedColliders.Length; i++)
        {
            Collider self = cachedColliders[i];
            if (self == null || !self.enabled || self.isTrigger)
                continue;

            for (int j = 0; j < otherColliders.Length; j++)
            {
                Collider other = otherColliders[j];
                if (other == null || !other.enabled || other.isTrigger || other == self)
                    continue;
                if (other.transform == transform || other.transform.IsChildOf(transform))
                    continue;

                Physics.IgnoreCollision(self, other, true);
                ignoredCollisionPairs.Add(new IgnoredCollisionPair { Self = self, Other = other });
            }
        }

        if (ignoredCollisionPairs.Count > 0)
            collisionRestoreRoutine = StartCoroutine(RestoreIgnoredCollisionsWhenClear(minimumSeconds));
    }

    private IEnumerator RestoreIgnoredCollisionsWhenClear(float minimumSeconds)
    {
        float elapsed = 0f;
        while (elapsed < minimumSeconds)
        {
            elapsed += Time.fixedDeltaTime;
            yield return new WaitForFixedUpdate();
        }

        while (IgnoredCollisionsStillOverlapping())
            yield return new WaitForFixedUpdate();

        collisionRestoreRoutine = null;
        RestoreIgnoredCollisions();
    }

    private bool IgnoredCollisionsStillOverlapping()
    {
        for (int i = 0; i < ignoredCollisionPairs.Count; i++)
        {
            Collider self = ignoredCollisionPairs[i].Self;
            Collider other = ignoredCollisionPairs[i].Other;
            if (self == null || other == null || !self.enabled || !other.enabled)
                continue;
            if (self.bounds.Intersects(other.bounds))
                return true;
        }

        return false;
    }

    private void RestoreIgnoredCollisions()
    {
        if (collisionRestoreRoutine != null)
        {
            StopCoroutine(collisionRestoreRoutine);
            collisionRestoreRoutine = null;
        }

        for (int i = 0; i < ignoredCollisionPairs.Count; i++)
        {
            Collider self = ignoredCollisionPairs[i].Self;
            Collider other = ignoredCollisionPairs[i].Other;
            if (self != null && other != null)
                Physics.IgnoreCollision(self, other, false);
        }

        ignoredCollisionPairs.Clear();
    }

    private void RefreshCachedColliders()
    {
        cachedColliders = GetComponentsInChildren<Collider>();
    }

    private void OnDisable()
    {
        RestoreIgnoredCollisions();
        if (graspJoint != null)
        {
            Destroy(graspJoint);
            graspJoint = null;
        }
    }
}

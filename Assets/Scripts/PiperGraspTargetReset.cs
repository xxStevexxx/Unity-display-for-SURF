using UnityEngine;

[DisallowMultipleComponent]
public sealed class PiperGraspTargetReset : MonoBehaviour
{
    [SerializeField] private float resetBelowY = -0.4f;
    private Rigidbody body;
    private Vector3 startPosition;
    private Quaternion startRotation;

    private void Awake()
    {
        body = GetComponent<Rigidbody>();
        startPosition = transform.position;
        startRotation = transform.rotation;
    }

    private void FixedUpdate()
    {
        if (transform.position.y < resetBelowY)
            ResetToStart();
    }

    public void MarkHeld()
    {
    }

    public void CaptureStartPose()
    {
        startPosition = transform.position;
        startRotation = transform.rotation;
    }

    public void ResetToStart()
    {
        transform.SetParent(null, true);
        transform.position = startPosition;
        transform.rotation = startRotation;

        if (body == null)
            body = GetComponent<Rigidbody>();
        if (body == null)
            return;

        if (!body.isKinematic)
        {
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
        }

        body.isKinematic = false;
        body.useGravity = true;
        body.WakeUp();
    }
}

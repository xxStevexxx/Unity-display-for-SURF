using System;
using System.Collections.Generic;
using UnityEngine;

public sealed class PiperUnityArticulationBackend : IPiperArmBackend
{
    private const int JointCount = 6;

    private readonly PiperArmStatus status = new();
    private readonly PiperJointCommand feedback = new();
    private PiperJointCommand lastCommand = new();
    private PiperArmController controller;
    private readonly List<ArticulationBody> revoluteJoints = new();
    private ArticulationBody gripperLeft;
    private ArticulationBody gripperRight;
    private ArticulationBody root;
    private Transform endEffector;
    private bool hasCartesianTarget;
    private Vector3 cartesianTargetPosition;

    public PiperArmStatus Status => status;
    public PiperJointCommand Feedback => feedback;
    public PiperJointCommand LastCommand => lastCommand;

    public void Initialize(PiperArmController owner)
    {
        controller = owner;
        revoluteJoints.Clear();

        foreach (var body in controller.GetComponentsInChildren<ArticulationBody>(true))
        {
            if (body.isRoot)
            {
                root = body;
                root.immovable = true;
                root.useGravity = false;
                continue;
            }

            if (body.jointType == ArticulationJointType.RevoluteJoint && revoluteJoints.Count < JointCount)
            {
                ApplyHardDrive(body);
                revoluteJoints.Add(body);
            }
            else if (body.jointType == ArticulationJointType.PrismaticJoint)
            {
                ApplyHardDrive(body);
                if (body.name == "link7")
                    gripperLeft = body;
                else if (body.name == "link8")
                    gripperRight = body;
            }

            if (body.name == "link6")
                endEffector = body.transform;
        }

        if (endEffector == null && revoluteJoints.Count > 0)
            endEffector = revoluteJoints[revoluteJoints.Count - 1].transform;

        feedback.EnsureArrays();
        lastCommand.EnsureArrays();
        for (int i = 0; i < JointCount; i++)
        {
            if (i >= revoluteJoints.Count)
                continue;

            feedback.positionRad[i] = ReadRevolutePositionRad(revoluteJoints[i]);
            lastCommand.positionRad[i] = feedback.positionRad[i];
        }

        feedback.SpeedPercent = controller.DefaultSpeedPercent;
        lastCommand.SpeedPercent = controller.DefaultSpeedPercent;
        feedback.GripperEffort = controller.GripperEffort;
        lastCommand.GripperEffort = controller.GripperEffort;
        status.runState = PiperArmRunState.Disabled;
        status.enabled = false;
        status.ctrlMode = 1;
        status.modeFeedback = 1;
        for (int i = 0; i < status.jointCommunicationOk.Length; i++)
            status.jointCommunicationOk[i] = true;
    }

    public void Tick(float deltaTime)
    {
        feedback.EnsureArrays();

        if (!status.enabled)
            hasCartesianTarget = false;

        for (int i = 0; i < revoluteJoints.Count && i < JointCount; i++)
        {
            feedback.positionRad[i] = ReadRevolutePositionRad(revoluteJoints[i]);
            feedback.velocity[i] = 0.0;
            feedback.effort[i] = 0.0;
        }

        feedback.GripperMeters = ReadGripperOpening();
        feedback.SpeedPercent = lastCommand.SpeedPercent;
        feedback.GripperEffort = lastCommand.GripperEffort;
        status.motionStatus = status.enabled ? (byte)1 : (byte)0;
    }

    public void Enable()
    {
        status.runState = PiperArmRunState.Enabled;
        status.enabled = true;
        status.armStatus = 1;

        if (root != null)
        {
            root.immovable = true;
            root.useGravity = false;
        }

        foreach (var joint in revoluteJoints)
            ApplyHardDrive(joint);

        if (gripperLeft != null)
            ApplyHardDrive(gripperLeft);
        if (gripperRight != null)
            ApplyHardDrive(gripperRight);
    }

    public void Disable()
    {
        status.runState = PiperArmRunState.Disabled;
        status.enabled = false;
        status.armStatus = 0;

        foreach (var joint in revoluteJoints)
            ApplySoftDrive(joint);

        if (gripperLeft != null)
            ApplySoftDrive(gripperLeft);
        if (gripperRight != null)
            ApplySoftDrive(gripperRight);
    }

    public void SendJointCommand(PiperJointCommand command)
    {
        command.EnsureArrays();
        lastCommand = command.Clone();
        hasCartesianTarget = false;
        status.ctrlMode = (byte)Mathf.Clamp(command.mode1, 0, 255);
        status.modeFeedback = (byte)Mathf.Clamp(command.mode2, 0, 255);

        if (!status.enabled)
            return;

        for (int i = 0; i < revoluteJoints.Count && i < JointCount; i++)
        {
            var joint = revoluteJoints[i];
            var drive = joint.xDrive;
            float targetDegrees = (float)(command.positionRad[i] * Mathf.Rad2Deg);
            drive.target = ClampToDriveLimits(drive, targetDegrees);
            drive.targetVelocity = 0f;
            joint.xDrive = drive;
        }

        ApplyGripper((float)command.GripperMeters);
    }

    public void SendEndPoseCommand(PiperEndPoseCommand command)
    {
        status.ctrlMode = (byte)Mathf.Clamp(command.mode1, 0, 255);
        status.modeFeedback = (byte)Mathf.Clamp(command.mode2, 0, 255);
        if (status.enabled)
            ApplyGripper((float)command.gripper);
    }

    public bool TryGetEndEffectorPose(out Vector3 position, out Quaternion rotation)
    {
        if (endEffector == null)
        {
            position = default;
            rotation = default;
            return false;
        }

        position = endEffector.position;
        rotation = endEffector.rotation;
        return true;
    }

    public bool TryMoveEndEffector(Vector3 worldDelta, int iterations, float gain, float maxJointStepDegrees)
    {
        if (!status.enabled || endEffector == null || revoluteJoints.Count == 0 || worldDelta.sqrMagnitude <= 0f)
            return false;

        if (!hasCartesianTarget || Vector3.Distance(cartesianTargetPosition, endEffector.position) > 0.25f)
        {
            cartesianTargetPosition = endEffector.position;
            hasCartesianTarget = true;
        }

        cartesianTargetPosition += worldDelta;
        Vector3 lead = cartesianTargetPosition - endEffector.position;
        const float maxTargetLeadMeters = 0.08f;
        if (lead.sqrMagnitude > maxTargetLeadMeters * maxTargetLeadMeters)
            cartesianTargetPosition = endEffector.position + lead.normalized * maxTargetLeadMeters;

        bool moved = false;
        int solveIterations = Mathf.Clamp(iterations, 1, 24);
        float solverGain = Mathf.Clamp(gain, 0.05f, 1f);
        float maxStepRad = Mathf.Max(0.1f, maxJointStepDegrees) * Mathf.Deg2Rad;
        float[] usedStepRad = new float[JointCount];

        for (int iteration = 0; iteration < solveIterations; iteration++)
        {
            Vector3 error = cartesianTargetPosition - endEffector.position;
            if (error.sqrMagnitude < 0.000001f)
                break;

            for (int i = revoluteJoints.Count - 1; i >= 0 && i < JointCount; i--)
            {
                ArticulationBody joint = revoluteJoints[i];
                Vector3 axis = joint.transform.TransformDirection(Vector3.right);
                Vector3 toEnd = endEffector.position - joint.transform.position;
                Vector3 jacobian = Vector3.Cross(axis, toEnd);
                float denominator = jacobian.sqrMagnitude + 0.0001f;
                float deltaRad = Vector3.Dot(jacobian, error) / denominator;
                float remainingStepRad = Mathf.Max(0f, maxStepRad - Mathf.Abs(usedStepRad[i]));
                if (remainingStepRad <= 0f)
                    continue;

                deltaRad *= solverGain * CartesianJointWeight(i);
                deltaRad = Mathf.Clamp(deltaRad, -remainingStepRad, remainingStepRad);
                if (Mathf.Abs(deltaRad) < 0.000001f)
                    continue;

                var drive = joint.xDrive;
                float previousTargetDegrees = drive.target;
                float targetDegrees = previousTargetDegrees + deltaRad * Mathf.Rad2Deg;
                drive.target = ClampToDriveLimits(drive, targetDegrees);
                drive.targetVelocity = 0f;
                joint.xDrive = drive;
                lastCommand.positionRad[i] = drive.target * Mathf.Deg2Rad;
                float appliedDeltaRad = (drive.target - previousTargetDegrees) * Mathf.Deg2Rad;
                usedStepRad[i] += Mathf.Abs(appliedDeltaRad);
                moved |= Mathf.Abs(appliedDeltaRad) >= 0.000001f;
            }

            Physics.SyncTransforms();
        }

        return moved;
    }

    private static float CartesianJointWeight(int jointIndex)
    {
        return jointIndex switch
        {
            5 => 0.08f,
            4 => 0.25f,
            3 => 0.55f,
            _ => 1f
        };
    }

    private void ApplyHardDrive(ArticulationBody joint)
    {
        joint.useGravity = false;
        joint.linearDamping = controller.LinearDamping;
        joint.angularDamping = controller.AngularDamping;
        joint.jointFriction = controller.JointFriction;

        var drive = joint.xDrive;
        drive.stiffness = controller.JointStiffness;
        drive.damping = controller.JointDamping;
        drive.forceLimit = controller.JointForceLimit;
        drive.targetVelocity = 0f;
        joint.xDrive = drive;
    }

    private void ApplySoftDrive(ArticulationBody joint)
    {
        joint.useGravity = true;
        joint.linearDamping = controller.LinearDamping;
        joint.angularDamping = controller.AngularDamping;
        joint.jointFriction = controller.JointFriction;

        var drive = joint.xDrive;
        drive.stiffness = controller.DisabledJointStiffness;
        drive.damping = controller.DisabledJointDamping;
        drive.forceLimit = controller.DisabledJointForceLimit;
        drive.targetVelocity = 0f;
        joint.xDrive = drive;
    }

    private void ApplyGripper(float openingMeters)
    {
        float opening = Mathf.Clamp(openingMeters, 0f, 0.08f);
        if (gripperLeft != null)
        {
            var drive = gripperLeft.xDrive;
            drive.target = Mathf.Clamp(opening * 0.5f, drive.lowerLimit, drive.upperLimit);
            gripperLeft.xDrive = drive;
        }

        if (gripperRight != null)
        {
            var drive = gripperRight.xDrive;
            drive.target = Mathf.Clamp(-opening * 0.5f, drive.lowerLimit, drive.upperLimit);
            gripperRight.xDrive = drive;
        }
    }

    private float ReadGripperOpening()
    {
        float left = gripperLeft != null ? Mathf.Abs(ReadPrismaticPositionMeters(gripperLeft)) : 0f;
        float right = gripperRight != null ? Mathf.Abs(ReadPrismaticPositionMeters(gripperRight)) : 0f;
        return left + right;
    }

    private static float ReadRevolutePositionRad(ArticulationBody joint)
    {
        if (joint == null)
            return 0f;

        var position = joint.jointPosition;
        if (position.dofCount > 0)
            return position[0];

        return joint.xDrive.target * Mathf.Deg2Rad;
    }

    private static float ReadPrismaticPositionMeters(ArticulationBody joint)
    {
        if (joint == null)
            return 0f;

        var position = joint.jointPosition;
        if (position.dofCount > 0)
            return position[0];

        return joint.xDrive.target;
    }

    private static float ClampToDriveLimits(ArticulationDrive drive, float target)
    {
        if (drive.lowerLimit < drive.upperLimit)
            return Mathf.Clamp(target, drive.lowerLimit, drive.upperLimit);

        return target;
    }
}

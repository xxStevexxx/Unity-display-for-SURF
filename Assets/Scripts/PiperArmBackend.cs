public interface IPiperArmBackend
{
    PiperArmStatus Status { get; }
    PiperJointCommand Feedback { get; }
    PiperJointCommand LastCommand { get; }

    void Initialize(PiperArmController controller);
    void Tick(float deltaTime);
    void Enable();
    void Disable();
    void SendJointCommand(PiperJointCommand command);
    void SendEndPoseCommand(PiperEndPoseCommand command);
}

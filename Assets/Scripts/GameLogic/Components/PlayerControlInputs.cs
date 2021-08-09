using Unity.Entities;

public struct PlayerControlInputs : IComponentData {
    public int moveDirection;
    public bool jumpPressed;
    public bool swapAttempted;
}

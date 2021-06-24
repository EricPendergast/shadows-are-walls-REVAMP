using Unity.Entities;

public struct PlayerComponent : IComponentData {
    public int moveDirection;
    public bool jumpPressed;
}
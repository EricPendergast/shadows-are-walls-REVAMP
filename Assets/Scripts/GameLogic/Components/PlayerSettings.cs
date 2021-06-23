using Unity.Entities;

[GenerateAuthoringComponent]
public struct PlayerSettings : IComponentData {
    public float softness;
    public float moveSpeed;
}

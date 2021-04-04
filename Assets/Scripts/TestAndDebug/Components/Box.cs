
using Unity.Entities;
using Unity.Mathematics;

[GenerateAuthoringComponent]
public struct Box : IComponentData {
    public float2 pos;
    public float rot;
    public float radius;
}

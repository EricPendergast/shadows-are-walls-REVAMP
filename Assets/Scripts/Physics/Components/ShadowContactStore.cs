using Unity.Entities;
using Unity.Mathematics;

public struct ShadowContactStore : IBufferElementData {
    public float2 point;
    public float2 normal;
    public Entity other;
}

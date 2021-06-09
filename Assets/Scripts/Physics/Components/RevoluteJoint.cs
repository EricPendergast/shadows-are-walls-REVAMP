using Unity.Entities;
using Unity.Mathematics;

public struct RevoluteJoint : IComponentData {
    public Entity e1;
    public Entity e2;
    public float2 r1;
    public float2 r2;

    public float softness;
    public float beta;
}

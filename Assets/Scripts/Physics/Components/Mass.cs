using Unity.Mathematics;
using Unity.Entities;

public struct Mass : IComponentData {
    public float mass;
    public float inertia;
    public float3 M => new float3(mass, mass, inertia);
    public float3 M_inv => new float3(1/mass, 1/mass, 1/inertia);
}

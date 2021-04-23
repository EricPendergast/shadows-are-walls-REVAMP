using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

public struct LightSource : IComponentData {
    public float2 pos;

    public float rot;
    public float inertia;
}

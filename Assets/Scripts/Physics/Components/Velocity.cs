using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

// TODO: Not used yet... But certain steps only
// need velocity, so I think its good to keep it a
// separate component
public struct Velocity : IComponentData {
    public float2 vel;
    public float angVel;
}

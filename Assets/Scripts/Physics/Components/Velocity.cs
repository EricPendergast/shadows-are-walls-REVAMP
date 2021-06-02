using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

// TODO: Not used yet... But certain steps only
// need velocity, so I think its good to keep it a
// separate component
public struct Velocity : IComponentData {
    public float2 vel;
    public float angVel;

    public static Velocity operator+(Velocity v1, Velocity v2) {
        return new Velocity{vel=v1.vel + v2.vel, angVel=v1.angVel + v2.angVel};
    }
    public static Velocity operator+(Velocity v, float3 other) {
        return new Velocity{vel = v.vel + other.xy, angVel = v.angVel + other.z};
    }

    public static Velocity operator/(Velocity v, float div) {
        return new Velocity{vel=v.vel/div, angVel=v.angVel/div};
    }

}

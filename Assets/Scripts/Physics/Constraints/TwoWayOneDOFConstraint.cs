using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

public struct RelativeVelocityManifold {
    public Entity e1;
    public Entity e2;
    public float2 r1;
    public float2 r2;
    public float2 normal;
    public float speedE1AlongNormal;
    public int id;
    public float softness;
}

public struct TwoWayOneDOFConstraint : IWarmStartConstraint<float> {
    // The opaque object
    public Entity e1;
    // The shadow hitting object
    public Entity e2;
    private float lambdaAccum;
    public float GetAccumulatedLambda() {
        return lambdaAccum;
    }

    public int id {get; set;}

    Float6 M_inv;

    OneDOFConstraint<Float6> constraint;

    public TwoWayOneDOFConstraint(in RelativeVelocityManifold m, ComponentDataFromEntity<Mass> masses, float dt) {
        e1 = m.e1;
        e2 = m.e2;
        id = m.id;

        M_inv = new Float6(masses[e1].M_inv, masses[e2].M_inv);

        lambdaAccum = 0;

        constraint = new OneDOFConstraint<Float6>(
            J: new Float6(
                    new float3(m.normal, Lin.Cross(m.r1, m.normal)), 
                    new float3(-m.normal, -Lin.Cross(m.r2, m.normal))
            ),
            M_inv: M_inv,
            bias: -m.speedE1AlongNormal,
            softness: m.softness
        );
    }

    public IConstraint Clone() {
        return this;
    }

    public void WarmStart(ComponentDataFromEntity<Velocity> vels, float dt, float prevLambda) {
        lambdaAccum = prevLambda;

        var v1 = vels[e1];
        var v2 = vels[e2];

        if (CollisionSystem.accumulateImpulses) {
            Float6 P_n = constraint.GetImpulse(lambdaAccum);

            ApplyImpulse(P_n, ref v1, ref v2);
        }

        vels[e1] = v1;
        vels[e2] = v2;
    }

    public void ApplyImpulses(ComponentDataFromEntity<Velocity> vels, float dt) {
        var v1 = vels[e1];
        var v2 = vels[e2];

        Float6 v = GetV(ref v1, ref v2);

        Float6 P = constraint.GetImpulse(v, ref lambdaAccum);

        ApplyImpulse(P, ref v1, ref v2);

        vels[e1] = v1;
        vels[e2] = v2;
    }

    private static Float6 GetV(ref Velocity v1, ref Velocity v2) {
        return new Float6(
            new float3(v1.vel, v1.angVel),
            new float3(v2.vel, v2.angVel)
        );
    }

    private void ApplyImpulse(Float6 impulse, ref Velocity v1, ref Velocity v2) {
        impulse = impulse.Mult(M_inv);

        v1.vel += impulse.v1.xy;
        v1.angVel += impulse.v1.z;

        v2.vel += impulse.v2.xy;
        v2.angVel += impulse.v2.z;
    }
}

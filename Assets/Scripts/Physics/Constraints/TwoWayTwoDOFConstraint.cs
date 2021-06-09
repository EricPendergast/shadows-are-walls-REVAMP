using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

public struct RevoluteJointManifold {
    public Entity e1;
    public Entity e2;
    public float2 r1;
    public float2 r2;
    public int id;
    public float2 delta;
    public float softness;
    public float beta;
}

public struct TwoWayTwoDOFConstraint : IConstraint<float2> {
    private Entity e1;
    private Entity e2;

    public int id {get;}
    private float2 lambdaAccum;

    public float2 GetAccumulatedLambda() {
        return lambdaAccum;
    }

    private TwoDOFConstraint<Float6> constraint;

    Float6 M_inv;

    public TwoWayTwoDOFConstraint(RevoluteJointManifold m, ComponentDataFromEntity<Mass> masses, float dt) {
        e1 = m.e1;
        e2 = m.e2;
        id = m.id;

        float2 dCdOmega1 = Lin.Cross(m.r1, 1);
        float2 dCdOmega2 = -Lin.Cross(m.r2, 1);

        M_inv = new Float6(masses[e1].M_inv, masses[e2].M_inv);

        constraint = new TwoDOFConstraint<Float6>(
            J1: new Float6(-1, 0, dCdOmega1.x, 1, 0, dCdOmega2.x),
            J2: new Float6(0, -1, dCdOmega1.y, 0, 1, dCdOmega2.y),
            M_inv: M_inv,
            bias: m.delta * m.beta / dt,
            softness: m.softness
        );
            
        lambdaAccum = float2.zero;
    }

    public void PreStep(ComponentDataFromEntity<Velocity> vels, float dt, float2 prevLambda) {
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

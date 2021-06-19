using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

using Lambda = Unity.Mathematics.float2;

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


public struct TwoWayTwoDOFConstraint : IWarmStartConstraint<Lambda> {
    private Entity e1;
    private Entity e2;

    public int id {get;}
    private Lambda lambdaAccum;

    public Lambda GetAccumulatedLambda() {
        return lambdaAccum;
    }

    private TwoDOFConstraint<Float6> constraint;

    public Float6 M_inv;

    public IConstraint Clone() {
        return this;
    }

    public TwoWayTwoDOFConstraint(RevoluteJointManifold m, ComponentDataFromEntity<Mass> masses, float dt) :
        this(m, new Float6(masses[m.e1].M_inv, masses[m.e2].M_inv), dt) {}

    public TwoWayTwoDOFConstraint(RevoluteJointManifold m, Float6 M_inv, float dt) {
        e1 = m.e1;
        e2 = m.e2;
        id = m.id;

        float2 normX = new float2(1, 0);
        float2 normY = new float2(0, 1);

        this.M_inv = M_inv;

        constraint = new TwoDOFConstraint<Float6>(
            J1: new Float6(
                new float3(-normX, -Lin.Cross(m.r1, normX)),
                new float3(normX, Lin.Cross(m.r2, normX))
            ),
            J2: new Float6(
                new float3(-normY, -Lin.Cross(m.r1, normY)),
                new float3(normY, Lin.Cross(m.r2, normY))
            ),
            M_inv: M_inv,
            bias: m.delta * m.beta / dt,
            softness: m.softness
        );

        lambdaAccum = float2.zero;
    }

    public void WarmStart(ComponentDataFromEntity<Velocity> vels, float dt, Lambda prevLambda) {
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

        ApplyImpulses(ref v1, ref v2, dt);

        vels[e1] = v1;
        vels[e2] = v2;
    }

    public void ApplyImpulses(ref Velocity v1, ref Velocity v2, float dt) {
        Float6 v = GetV(ref v1, ref v2);

        Float6 P = constraint.GetImpulse(v, ref lambdaAccum);

        ApplyImpulse(P, ref v1, ref v2);
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

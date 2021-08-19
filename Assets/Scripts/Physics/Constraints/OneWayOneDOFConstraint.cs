using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

public struct TargetAngularVelocityManifold {
    public Entity e;
    public int id;
    public float targetAngVel;
    public float softness;

    public static float AngVelForTargetOffset(float offset, float maxSpeed, float dt) {
        float rotationOffsetSign = math.sign(offset);
        return rotationOffsetSign * math.min(math.abs(offset/dt), maxSpeed);
    }
}

public struct TargetVelocityManifold {
    public Entity e;
    public float2 r;
    public int id;
    public float2 normal;
    public float targetSpeed;
    public float softness;
}

public struct OneWayOneDOFConstraint : IWarmStartConstraint<float> {
    private Entity e;

    public int id {get;}
    private float lambdaAccum;

    public float GetAccumulatedLambda() {
        return lambdaAccum;
    }

    private OneDOFConstraint<Float3> constraint;

    public Float3 M_inv;

    public IConstraint Clone() {
        return this;
    }

    public OneWayOneDOFConstraint(TargetAngularVelocityManifold m, ComponentDataFromEntity<Mass> masses, float dt) :
        this(m, masses[m.e].M_inv, dt) {}

    public OneWayOneDOFConstraint(TargetAngularVelocityManifold m, Float3 M_inv, float dt) {
        e = m.e;
        id = m.id;

        this.M_inv = M_inv;

        constraint = new OneDOFConstraint<Float3>(
            J: new float3(0, 0, math.sign(m.targetAngVel)),
            M_inv: M_inv,
            bias: -math.abs(m.targetAngVel),
            softness: m.softness
        );

        lambdaAccum = 0;
    }

    public OneWayOneDOFConstraint(TargetVelocityManifold m, ComponentDataFromEntity<Mass> masses, float dt) :
        this(m, masses[m.e].M_inv, dt) {}

    public OneWayOneDOFConstraint(TargetVelocityManifold m, Float3 M_inv, float dt) {
        e = m.e;
        id = m.id;

        this.M_inv = M_inv;

        constraint = new OneDOFConstraint<Float3>(
            J: new float3(m.normal, Lin.Cross(m.r, m.normal)),
            M_inv: M_inv,
            bias: -m.targetSpeed,
            softness: m.softness
        );

        lambdaAccum = 0;
    }

    public void WarmStart(ComponentDataFromEntity<Velocity> vels, float dt, float prevLambda) {
        lambdaAccum = prevLambda;

        var v = vels[e];

        if (CollisionSystem.accumulateImpulses) {
            Float3 P_n = constraint.GetImpulse(lambdaAccum);

            ApplyImpulse(P_n, ref v);
        }

        vels[e] = v;
    }

    public void ApplyImpulses(ComponentDataFromEntity<Velocity> vels, float dt) {
        var v = vels[e];

        ApplyImpulses(ref v, dt);

        vels[e] = v;
    }

    public void ApplyImpulses(ref Velocity v, float dt) {
        Float3 P = constraint.GetImpulse(new float3(v.vel, v.angVel), ref lambdaAccum);

        ApplyImpulse(P, ref v);
    }

    private void ApplyImpulse(Float3 impulse, ref Velocity v) {
        impulse = impulse.Mult(M_inv);

        v.vel += impulse.v.xy;
        v.angVel += impulse.v.z;
    }
}

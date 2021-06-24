using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

public struct ShadowEdgeManifold {
    public float delta;
    public float2 n;
    // The opaque object
    //public Entity e1;
    public float2 x1;
    public float2 d1;
    // The shadow hitting object
    public Entity e2;
    public float2 x2;

    // Contact point
    public float2 p;
    // Unique id for the point of contact on the shadow hitting object.
    public int contactIdOn2;
}

public struct RelativeVelocityManifold {
    public Entity e1;
    public Entity e2;
    public float2 r1;
    public float2 r2;
    public float2 normal;
    public float minSpeedE1AlongNormal;
    public int id;
    public float softness;
}

public struct TwoWayPenConstraint : IWarmStartConstraint<float> {
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

    PenetrationConstraint<Float6> penConstraint;

    public TwoWayPenConstraint(in RelativeVelocityManifold m, ComponentDataFromEntity<Mass> masses, float dt) {
        e1 = m.e1;
        e2 = m.e2;
        id = m.id;

        M_inv = new Float6(masses[e1].M_inv, masses[e2].M_inv);

        lambdaAccum = 0;

        penConstraint = new PenetrationConstraint<Float6>(
            J: new Float6(
                    new float3(m.normal, Lin.Cross(m.r1, m.normal)), 
                    new float3(-m.normal, -Lin.Cross(m.r2, m.normal))
            ),
            M_inv: M_inv,
            bias: -m.minSpeedE1AlongNormal,
            softness: m.softness
        );
    }

    public TwoWayPenConstraint(in Partial p, ComponentDataFromEntity<Mass> masses, float dt) :
        this(in p, masses, dt, CollisionSystem.positionCorrection ? .1f : 0, -.01f) {}

    public TwoWayPenConstraint(in Partial p, ComponentDataFromEntity<Mass> masses, float dt, float beta, float delta_slop) {
        e1 = p.e1;
        e2 = p.e2;

        Float6 M_inv = new Float6(masses[e1].M_inv, masses[e2].M_inv);
        id = p.id;

        float bias = 0;

        if (p.delta < delta_slop) {
            bias = beta * (p.delta - delta_slop) / dt;
        }

        penConstraint = new PenetrationConstraint<Float6>(p.J_n, M_inv, bias);

        this.M_inv = M_inv;

        lambdaAccum = new float();
    }

    public IConstraint Clone() {
        return this;
    }

    public struct Partial {
        public Entity e1;
        public Entity e2;
        public int id;

        public Float6 J_n;
        public float delta;


        public Partial(in Prototype p, in ShadowCornerCalculator.EdgeMount mount, in ShadowEdgeManifold m) {
            e1 = mount.castingEntity;
            e2 = m.e2;
            id = new int2(m.contactIdOn2, mount.id).GetHashCode();
            J_n = p.J_n;
            delta = p.delta;
            if (mount.castingShapeType == EdgeSourceType.Box) {
                float2 velMult = Lin.Cross(1, mount.point - m.x1)/math.lengthsq(mount.point - m.x1);
                float angVelMult = math.dot(mount.point - mount.shapeCenter, mount.point - m.x1)/math.lengthsq(mount.point - m.x1);
                J_n.v1 = new float3(
                    J_n.v1.z * velMult,
                    J_n.v1.z * angVelMult
                );
                // The previous version of this code:
                // {
                //     float thing = math.dot(mount.point - m.x1, m.p - m.x1) / math.lengthsq(mount.point - m.x1);
                    
                //     J_n = new Float6(
                //         new float3(m.n*thing, Lin.Cross(mount.point-mount.shapeCenter, m.n)*thing),
                //         new float3(-m.n, -Lin.Cross(m.p-m.x2, m.n))
                //     );
                // }
            } else {
                // Do nothing
            }
        }

        public struct Prototype {
            public Float6 J_n;
            public float delta;

            public Prototype(in ShadowEdgeManifold m) {
                J_n = new Float6(
                    new float3(0, 0, Lin.Cross(m.p - m.x1, m.n)), 
                    new float3(-m.n, -Lin.Cross(m.p - m.x2, m.n))
                );

                delta = m.delta;
            }
        }
    }

    public void WarmStart(ComponentDataFromEntity<Velocity> vels, float dt, float prevLambda) {
        lambdaAccum = prevLambda;

        var v1 = vels[e1];
        var v2 = vels[e2];

        if (CollisionSystem.accumulateImpulses) {
            Float6 P_n = penConstraint.GetImpulse(lambdaAccum);

            ApplyImpulse(P_n, ref v1, ref v2);
        }

        vels[e1] = v1;
        vels[e2] = v2;
    }

    public void ApplyImpulses(ComponentDataFromEntity<Velocity> vels, float dt) {
        var v1 = vels[e1];
        var v2 = vels[e2];

        Float6 v = GetV(ref v1, ref v2);

        Float6 P = penConstraint.GetImpulse(v, ref lambdaAccum);

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

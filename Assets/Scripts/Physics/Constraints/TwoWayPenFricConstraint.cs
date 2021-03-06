using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

// Constraint for penetration and friction. With the correct
// jacobian, should work for normal rigidbodies (like box, circle,
// etc). May or may not work for crazier rigidbodies.
public struct TwoWayPenFricConstraint : IWarmStartConstraint<LambdaNT> {
    public Entity e1 {get;}
    public Entity e2 {get;}
    private LambdaNT lambdaAccum;
    public LambdaNT GetAccumulatedLambda() {
        return lambdaAccum;
    }

    public int id {get;}

    public Float6 M_inv;

    PenetrationConstraint<Float6> penConstraint;
    FrictionConstraint<Float6> fricConstraint;

    public TwoWayPenFricConstraint(in Partial p, ComponentDataFromEntity<Mass> masses, float dt, float beta, float delta_slop, float friction) :
        this(
            in p,
            new Float6(masses[p.e1].M_inv, masses[p.e2].M_inv),
            dt: dt,
            beta: beta,
            delta_slop: delta_slop,
            friction: friction
        ) {}

    public TwoWayPenFricConstraint(in TwoWayPenFricConstraint previous, in Partial p, float dt, float beta, float delta_slop) :
    this(
        in p,
        previous.M_inv,
        dt: dt,
        beta: beta,
        delta_slop: delta_slop,
        friction: previous.fricConstraint.frictionCoeff
    ) {}

    private TwoWayPenFricConstraint(in Partial p, Float6 M_inv, float dt, float beta, float delta_slop, float friction) {
        e1 = p.e1;
        e2 = p.e2;

        this.M_inv = M_inv;
        id = p.id;

        float bias = 0;
        
        if (p.delta < delta_slop) {
            bias = beta * (p.delta - delta_slop) / dt;
        }

        penConstraint = new PenetrationConstraint<Float6>(p.J_n, M_inv, bias);
        fricConstraint = new FrictionConstraint<Float6>(p.J_t, M_inv, friction);

        lambdaAccum = new LambdaNT();
    }

    public TwoWayPenFricConstraint(in Partial p, ComponentDataFromEntity<Mass> masses, float dt, float friction) :
        this(in p, masses, dt, CollisionSystem.positionCorrection ? .1f : 0, -.01f, friction) {}

    public IConstraint Clone() {
        return this;
    }

    public struct Partial {
        public Entity e1;
        public Entity e2;
        public int id;

        public Float6 J_n;
        public Float6 J_t;
        public float delta;

        public Partial(Entity e1, Entity e2, Position pos1, Position pos2, Geometry.Manifold m, bool useContact1) {
            this.e1 = e1;
            this.e2 = e2;
            var contactWithId = useContact1 ? m.contact1 : (Geometry.Contact)m.contact2;

            float2 contact = contactWithId.point;
            id = contactWithId.id.GetHashCode();

            { // Normal precomputation
                J_n = new Float6(
                    new float3(-m.normal, -Lin.Cross(contact-pos1.pos, m.normal)), 
                    new float3(m.normal, Lin.Cross(contact-pos2.pos, m.normal))
                );

                delta = -m.overlap;
            }


            { // Tangent precomputation (friction)
                float2 tangent = Lin.Cross(m.normal, -1);

                J_t = new Float6(
                    new float3(tangent, Lin.Cross(contact-pos1.pos, tangent)), 
                    new float3(-tangent, -Lin.Cross(contact-pos2.pos, tangent)));
            }

        }

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
            } else {
                // Do nothing
            }

            float2 tangent = Lin.Cross(m.n, -1);

            J_t = new Float6(
                new float3(tangent, Lin.Cross(mount.point - m.x1, tangent)),
                new float3(-tangent, -Lin.Cross(m.p - m.x2, tangent))
            );
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

    public void WarmStart(ComponentDataFromEntity<Velocity> vels, float dt, LambdaNT prevLambdas) {

        lambdaAccum = prevLambdas;

        Velocity v1 = vels[e1];
        Velocity v2 = vels[e2];

        if (CollisionSystem.accumulateImpulses) {
            Float6 P_n = penConstraint.GetImpulse(lambdaAccum.n);
            Float6 P_t = fricConstraint.GetImpulse(lambdaAccum.t);

            ApplyImpulse(P_n.Add(P_t), ref v1, ref v2);
        }

        vels[e1] = v1;
        vels[e2] = v2;
    }

    public void ApplyImpulses(ComponentDataFromEntity<Velocity> vels, float dt) {
        Velocity v1 = vels[e1];
        Velocity v2 = vels[e2];

        {
            Float6 v = GetV(ref v1, ref v2);

            Float6 P = fricConstraint.GetImpulse(v, ref lambdaAccum.t, ref lambdaAccum.n);

            ApplyImpulse(P, ref v1, ref v2);
        }

        {
            Float6 v = GetV(ref v1, ref v2);

            Float6 P = penConstraint.GetImpulse(v, ref lambdaAccum.n);

            ApplyImpulse(P, ref v1, ref v2);
        }

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

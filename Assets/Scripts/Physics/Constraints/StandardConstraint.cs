using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

using ContactId = Physics.Math.Geometry.ContactId;

// TODO: Put this in StandardConstraint
public struct Lambdas {
    public float n;
    public float t;
}

// Constraint for penetration and friction. With the correct
// jacobian, should work for normal rigidbodies (like box, circle,
// etc). May or may not work for crazier rigidbodies.
public struct TwoWayPenFricConstraint : IConstraint {
    public Entity e1 {get;}
    public Entity e2 {get;}
    private Lambdas accum;
    public Lambdas GetAccumulatedLambdas() {
        return accum;
    }

    public int id {get;}

    Float6 M_inv;

    PenetrationConstraint<Float6> penConstraint;
    FrictionConstraint<Float6> fricConstraint;

    public TwoWayPenFricConstraint(in Partial p, ComponentDataFromEntity<Mass> masses, float dt) {
        e1 = p.e1;
        e2 = p.e2;

        M_inv = new Float6(masses[e1].M_inv, masses[e2].M_inv);
        id = p.id;
        penConstraint = new PenetrationConstraint<Float6>(p.J_n, M_inv, p.bias/dt);
        fricConstraint = new FrictionConstraint<Float6>(p.J_t, M_inv, CollisionSystem.globalFriction);

        accum = new Lambdas();
    }

    public struct Partial {
        public Entity e1;
        public Entity e2;
        public int id;

        public Float6 J_n;
        public Float6 J_t;
        public float bias;

        public Partial(Entity e1, Entity e2, Box b1, Box b2, Geometry.Manifold m, bool useContact1) {
            this.e1 = e1;
            this.e2 = e2;
            var contactWithId = useContact1 ? m.contact1 : (Geometry.Contact)m.contact2;

            float2 contact = contactWithId.point;
            id = contactWithId.id.GetHashCode();

            { // Normal precomputation
                J_n = new Float6(
                    new float3(-m.normal, -Lin.Cross(contact-b1.pos, m.normal)), 
                    new float3(m.normal, Lin.Cross(contact-b2.pos, m.normal))
                );

                float delta = -m.overlap;
                float beta = CollisionSystem.positionCorrection ? .1f : 0;
                float delta_slop = -.01f;

                bias = 0;

                if (delta < delta_slop) {
                    bias = beta * (delta - delta_slop);
                }
            }


            { // Tangent precomputation (friction)
                float2 tangent = Lin.Cross(m.normal, -1);

                J_t = new Float6(
                    new float3(tangent, Lin.Cross(contact-b1.pos, tangent)), 
                    new float3(-tangent, -Lin.Cross(contact-b2.pos, tangent)));
            }

        }
    }

    public void PreStep(ref Velocity v1, ref Velocity v2, float dt, Lambdas prevLambdas) {
        accum = prevLambdas;

        if (CollisionSystem.accumulateImpulses) {
            Float6 P_n = penConstraint.GetImpulse(accum.n);
            Float6 P_t = fricConstraint.GetImpulse(accum.t);

            ApplyImpulse(P_n.Add(P_t), ref v1, ref v2);
        }
    }

    public void ApplyImpulse(ref Velocity v1, ref Velocity v2, float dt) {
        {
            Float6 v = GetV(ref v1, ref v2);

            Float6 P = fricConstraint.GetImpulse(v, ref accum.t, ref accum.n);

            ApplyImpulse(P, ref v1, ref v2);
        }

        {
            Float6 v = GetV(ref v1, ref v2);

            Float6 P = penConstraint.GetImpulse(v, ref accum.n);

            ApplyImpulse(P, ref v1, ref v2);
        }

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

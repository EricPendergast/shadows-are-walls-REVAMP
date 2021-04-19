using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

using ContactId = Physics.Math.Geometry.ContactId;

public struct Lambdas {
    public float n;
    public float t;
}

public struct BoxBoxConstraint : Constraint {
    public Entity box1 {get;}
    public Entity box2 {get;}
    private Lambdas accum;
    public Lambdas GetAccumulatedLambdas() {
        return accum;
    }
    public float2 normal {get;}
    public float2 contact {get;}

    public ContactId id {get;}

    Float6 M_inv;

    PenetrationConstraint<Float6> penConstraint;
    FrictionConstraint<Float6> fricConstraint;

    public BoxBoxConstraint(Entity box1, Entity box2, Geometry.Manifold manifold, bool useContact1) {
        this.box1 = box1;
        this.box2 = box2;
        this.normal = manifold.normal;
        var contact = useContact1 ? manifold.contact1 : (Geometry.Contact)manifold.contact2;
        this.contact = contact.point;
        this.id = contact.id;

        accum = new Lambdas();

        M_inv = default(Float6);

        penConstraint = new PenetrationConstraint<Float6>();
        fricConstraint = new FrictionConstraint<Float6>();
    }

    public void PreStep(Box box1, Box box2, ref Velocity v1, ref Velocity v2, float dt, Lambdas prevLambdas) {
        accum = prevLambdas;

        M_inv = new Float6(
            1/box1.mass, 1/box1.mass, 1/box1.inertia,
            1/box2.mass, 1/box2.mass, 1/box2.inertia
        );

        { // Normal precomputation
            Float6 J_n = new Float6(
                new float3(-normal, -Lin.Cross(contact-box1.pos, normal)), 
                new float3(normal, Lin.Cross(contact-box2.pos, normal))
            );

            float delta = -Geometry.GetOverlapOnAxis(box1.ToRect(), box2.ToRect(), normal);
            float beta = CollisionSystem.positionCorrection ? .1f : 0;
            float delta_slop = -.01f;

            float bias = 0;

            if (delta < delta_slop) {
                bias = (beta/dt) * (delta - delta_slop);
            }

            penConstraint = new PenetrationConstraint<Float6>(J_n, M_inv, bias);
        }


        { // Tangent precomputation (friction)
            float2 tangent = Lin.Cross(normal, -1);

            Float6 J_t = new Float6(
                new float3(tangent, Lin.Cross(contact-box1.pos, tangent)), 
                new float3(-tangent, -Lin.Cross(contact-box2.pos, tangent)));

            fricConstraint = new FrictionConstraint<Float6>(J_t, M_inv, CollisionSystem.globalFriction);
        }

        if (CollisionSystem.accumulateImpulses) {
            Float6 P_n = penConstraint.GetImpulse(accum.n);
            Float6 P_t = fricConstraint.GetImpulse(accum.t);

            ApplyImpulse(P_n.Add(P_t), ref v1, ref v2);
        }
    }

    public void ApplyImpulse(ref Velocity v1, ref Velocity v2, float dt) {
        float lambda;
        {
            Float6 v = GetV(ref v1, ref v2);

            lambda = penConstraint.GetLambda(v, ref accum.n);
            Float6 P = penConstraint.GetImpulse(lambda);

            ApplyImpulse(P, ref v1, ref v2);
        }

        {
            Float6 v = GetV(ref v1, ref v2);

            Float6 P = fricConstraint.GetImpulse(v, lambda, ref accum.t, ref accum.n);

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

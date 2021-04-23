using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

using ContactId = Physics.Math.Geometry.ContactId;

public struct ShadowEdgeConstraint : IConstraint {
    // The non opaque object
    public Entity e1 {get;}
    // The opaque object
    public Entity e2 {get;}
    private Lambdas accum;
    public Lambdas GetAccumulatedLambdas() {
        return accum;
    }
    public float2 normal {get;}
    public float2 contact {get => contact1;}
    // The contact on e1
    public float2 contact1 {get;}
    // The contact on e2 (the origin of the shadow edge)
    public float2 contact2 {get;}
    public float overlap;

    public ContactId id {get;}

    Float6 M_inv;

    PenetrationConstraint<Float6> penConstraint;

    public ShadowEdgeConstraint(Entity e1, Entity e2, Box box1, Box box2, float2 shadowOrigin, float2 lightOrigin, Geometry.Manifold manifold, bool useContact1, float dt) {
        this.e1 = e1;
        this.e2 = e2;
        this.normal = manifold.normal;
        { 
            var contact = useContact1 ? manifold.contact1 : (Geometry.Contact)manifold.contact2;
            this.contact1 = contact.point;
            this.id = contact.id;
        }
        this.contact2 = shadowOrigin;
        this.overlap = manifold.overlap;

        accum = new Lambdas();

        M_inv = new Float6(
            1/box1.mass, 1/box1.mass, 1/box1.inertia,
            1/box2.mass, 1/box2.mass, 1/box2.inertia
        );

        { // Normal precomputation

            float thing = math.dot(contact2 - lightOrigin, contact1 - lightOrigin) / math.lengthsq(contact2 - lightOrigin);
            Float6 J_n = new Float6(
                new float3(-normal, -Lin.Cross(contact1-box1.pos, normal)), 
                new float3(normal*thing, Lin.Cross(contact2-box2.pos, normal))
            );

            float delta = -overlap;
            float beta = CollisionSystem.positionCorrection ? .1f : 0;
            float delta_slop = -.01f;

            float bias = 0;

            if (delta < delta_slop) {
                bias = (beta/dt) * (delta - delta_slop);
            }

            penConstraint = new PenetrationConstraint<Float6>(J_n, M_inv, bias);
        }
    }

    public void PreStep(ref Velocity v1, ref Velocity v2, float dt, Lambdas prevLambdas) {
        accum = prevLambdas;

        if (CollisionSystem.accumulateImpulses) {
            Float6 P_n = penConstraint.GetImpulse(accum.n);

            ApplyImpulse(P_n, ref v1, ref v2);
        }
    }

    public void ApplyImpulse(ref Velocity v1, ref Velocity v2, float dt) {
        Float6 v = GetV(ref v1, ref v2);

        Float6 P = penConstraint.GetImpulse(v, ref accum.n);

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

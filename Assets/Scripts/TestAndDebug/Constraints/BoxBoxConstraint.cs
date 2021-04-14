using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

using ContactId = Physics.Math.Geometry.ContactId;

public struct Lambdas {
    public float n;
    public float t;
}

public struct BoxBoxConstraint {
    public Entity box1 {get;}
    public Entity box2 {get;}
    private Lambdas accum;
    public Lambdas GetAccumulatedLambdas() {
        return accum;
    }
    public float2 normal {get;}
    public float2 contact {get;}

    public ContactId id {get;}

    float3 M1_inv;
    float3 M2_inv;

    PenetrationConstraint pc;
    FrictionConstraint fc;

    public BoxBoxConstraint(Entity box1, Entity box2, float2 normal, Geometry.Contact contact) {
        this.box1 = box1;
        this.box2 = box2;
        this.normal = normal;
        this.contact = contact.point;
        id = contact.id;

        accum = new Lambdas();

        M1_inv = 0;
        M2_inv = 0;

        pc = new PenetrationConstraint();
        fc = new FrictionConstraint();
    }

    public void PreStep(ref ComponentDataFromEntity<Box> boxes, float dt, Lambdas prevLambdas) {
        accum = prevLambdas;

        Box box1 = boxes[this.box1];
        Box box2 = boxes[this.box2];

        M1_inv = new float3(1/box1.mass, 1/box1.mass, 1/box1.inertia);
        M2_inv = new float3(1/box2.mass, 1/box2.mass, 1/box2.inertia);

        { // Normal precomputation
            float3 J1_n = new float3(-normal, -Lin.Cross(contact-box1.pos, normal));
            float3 J2_n = new float3(normal, Lin.Cross(contact-box2.pos, normal));

            float delta = -Geometry.GetOverlapOnAxis(box1.ToRect(), box2.ToRect(), normal);
            float beta = CollisionSystem.positionCorrection ? .1f : 0;
            float delta_slop = -.01f;

            float bias = 0;

            if (delta < delta_slop) {
                bias = (beta/dt) * (delta - delta_slop);
            }

            pc = new PenetrationConstraint(J1_n, J2_n, M1_inv, M2_inv, bias);
        }


        { // Tangent precomputation (friction)
            float2 tangent = Lin.Cross(normal, -1);

            float3 J1_t = new float3(tangent, Lin.Cross(contact-box1.pos, tangent));
            float3 J2_t = new float3(-tangent, -Lin.Cross(contact-box2.pos, tangent));

            fc = new FrictionConstraint(J1_t, J2_t, M1_inv, M2_inv, CollisionSystem.globalFriction);
        }

        if (CollisionSystem.accumulateImpulses) {
            (float3 P1_n, float3 P2_n) = pc.GetImpulse(accum.n);
            (float3 P1_t, float3 P2_t) = fc.GetImpulse(accum.t);

            ApplyImpulse(P1_n + P1_t, ref box1);
            ApplyImpulse(P2_n + P2_t, ref box2);
        }

        boxes[this.box1] = box1;
        boxes[this.box2] = box2;
    }

    public void ApplyImpulse(ref ComponentDataFromEntity<Box> boxes, float dt) {
        Box box1 = boxes[this.box1];
        Box box2 = boxes[this.box2];


        float lambda;
        {
            float3 v1 = new float3(box1.vel, box1.angVel);
            float3 v2 = new float3(box2.vel, box2.angVel);

            lambda = pc.GetLambda(v1, v2, ref accum.n);
            (float3 P_n1, float3 P_n2) = pc.GetImpulse(lambda);

            ApplyImpulse(P_n1, ref box1);
            ApplyImpulse(P_n2, ref box2);
        }

        ////////////////////////////////
        /////////// Friction ///////////
        ////////////////////////////////

        {
            float3 v1 = new float3(box1.vel, box1.angVel);
            float3 v2 = new float3(box2.vel, box2.angVel);

            (float3 P_t1, float3 P_t2) = fc.GetImpulse(v1, v2, lambda, ref accum.t, ref accum.n);

            ApplyImpulse(P_t1, ref box1);
            ApplyImpulse(P_t2, ref box2);
        }

        // NOTE: This is not thread safe
        boxes[this.box1] = box1;
        boxes[this.box2] = box2;
    }

    private static void ApplyImpulse(float3 impulse, ref Box box) {
        float3 dv = new float3(1/box.mass, 1/box.mass, 1/box.inertia) * impulse;

        box.vel += dv.xy;
        box.angVel += dv.z;
    }
}

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

    float3 J1_t;
    float3 J2_t;

    float m_c_t;

    PenetrationConstraint pc;

    public BoxBoxConstraint(Entity box1, Entity box2, float2 normal, Geometry.Contact contact) {
        this.box1 = box1;
        this.box2 = box2;
        this.normal = normal;
        this.contact = contact.point;
        id = contact.id;

        accum = new Lambdas();

        M1_inv = 0;
        M2_inv = 0;
        J1_t = 0;
        J2_t = 0;
        m_c_t = 0;

        pc = new PenetrationConstraint();
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

            J1_t = new float3(tangent, Lin.Cross(contact-box1.pos, tangent));
            J2_t = new float3(-tangent, -Lin.Cross(contact-box2.pos, tangent));

            m_c_t = 1 / (math.dot(J1_t * M1_inv, J1_t) + math.dot(J2_t * M2_inv, J2_t));
        }

        if (CollisionSystem.accumulateImpulses) {
            (float3 P1_n, float3 P2_n) = pc.GetImpulse(accum.n);
            float3 P1_t = J1_t*accum.t;
            float3 P2_t = J2_t*accum.t;

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

            float lambda_t = -m_c_t * (math.dot(J1_t, v1) + math.dot(J2_t, v2));

            // Frictional coefficient
            float mu = CollisionSystem.globalFriction;

            ClampLambda_t(ref lambda_t, lambda,  mu, ref accum.t, ref accum.n);

            float3 P_t1 = J1_t*lambda_t;
            float3 P_t2 = J2_t*lambda_t;

            ApplyImpulse(P_t1, ref box1);
            ApplyImpulse(P_t2, ref box2);
        }

        // NOTE: This is not thread safe
        boxes[this.box1] = box1;
        boxes[this.box2] = box2;
    }

    private static void ApplyImpulse(float3 impulse, ref Box box) {
        // Delta velocity
        float3 dv = new float3(1/box.mass, 1/box.mass, 1/box.inertia) * impulse;

        box.vel += dv.xy;
        box.angVel += dv.z;
    }


    private static void ClampLambda(ref float lambda, ref float accumulated) {
        if (CollisionSystem.accumulateImpulses) {
            float oldAccumulated = accumulated;
            accumulated = math.max(accumulated + lambda, 0);
            lambda = accumulated - oldAccumulated;
        } else {
            lambda = math.max(lambda, 0);
        }
    }

    private static void ClampLambda_t(ref float lambda_t, float lambda_n, float frictionCoefficient, ref float accum_t, ref float accum_n) {
        if (CollisionSystem.accumulateImpulses) {
            float old_tAccumulated = accum_t;
            accum_t = math.clamp(accum_t + lambda_t, -accum_n*frictionCoefficient, accum_n*frictionCoefficient);
            lambda_t = accum_t - old_tAccumulated;
        } else {
            lambda_t = math.clamp(lambda_t, -lambda_n*frictionCoefficient, lambda_n*frictionCoefficient);
        }
    }
}

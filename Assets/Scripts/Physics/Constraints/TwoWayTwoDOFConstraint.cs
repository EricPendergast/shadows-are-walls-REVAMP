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

        float2 oneCrossR1 = Lin.Cross(1, m.r1);
        float2 oneCrossR2 = Lin.Cross(1, m.r2);

        this.M_inv = M_inv;

        var normal_n = normY;
        Float6 J_n = new Float6(
            new float3(-normal_n, -Lin.Cross(m.r1, normal_n)), 
            new float3(normal_n, Lin.Cross(m.r2, normal_n))
        );

        var normal_t = normX;
        Float6 J_t = new Float6(
            new float3(-normal_t, -Lin.Cross(m.r1, normal_t)), 
            new float3(normal_t, Lin.Cross(m.r2, normal_t))
        );

        float2x2 K1;
        K1.c0.x = M_inv.v1.x + M_inv.v2.x;	        K1.c1.x = 0.0f;
        K1.c0.y = 0.0f;								K1.c1.y = M_inv.v1.x + M_inv.v2.x;

        float2 r1 = m.r1;
        float2 r2 = m.r2;
        float2x2 K2;
        K2.c0.x =  M_inv.v1.z * r1.y * r1.y;		K2.c1.x = -M_inv.v1.z * r1.x * r1.y;
        K2.c0.y = -M_inv.v1.z * r1.x * r1.y;		K2.c1.y =  M_inv.v1.z * r1.x * r1.x;

        float2x2 K3;
        K3.c0.x =  M_inv.v2.z * r2.y * r2.y;		K3.c1.x = -M_inv.v2.z * r2.x * r2.y;
        K3.c0.y = -M_inv.v2.z * r2.x * r2.y;		K3.c1.y =  M_inv.v2.z * r2.x * r2.x;

        float2x2 K = K1 + K2 + K3;


        float2x2 myK = new float2x2();

        var J1 = J_n;
        var J2 = J_t;

        myK.c0.x = J2.Mult(M_inv).Dot(J2);    myK.c1.x = J1.Mult(M_inv).Dot(J2);
        myK.c0.y = J2.Mult(M_inv).Dot(J2);    myK.c1.y = J1.Mult(M_inv).Dot(J1);

        //constraint = new OneDOFConstraint<Float6>(
        //    //J1: new Float6(
        //    //    new float3(-normX, 0),// -Lin.Cross(m.r1, normX)),
        //    //    new float3(normX, 0)// Lin.Cross(m.r2, normX))
        //    //),
        //    J: J_n,
        //    //J: new Float6(
        //    //    new float3(-normY, -Lin.Cross(m.r1, normY)),
        //    //    new float3(normY, Lin.Cross(m.r2, normY))
        //    //),
        //    //J1: new Float6(-1, 0, -oneCrossR1.x, 1, 0, oneCrossR2.x),
        //    //J: new Float6(0, -1, -oneCrossR1.y, 0, 1, oneCrossR2.y),
        //    M_inv: M_inv,
        //    //bias: m.delta * m.beta / dt,
        //    bias: math.dot(m.delta, normal_n) * m.beta / dt,
        //    softness: m.softness
        //);
        constraint = new TwoDOFConstraint<Float6>(
            //J1: new Float6(
            //    new float3(-normX, 0),// -Lin.Cross(m.r1, normX)),
            //    new float3(normX, 0)// Lin.Cross(m.r2, normX))
            //),
            J1: J_n,
            J2: J_t,
            //J: new Float6(
            //    new float3(-normY, -Lin.Cross(m.r1, normY)),
            //    new float3(normY, Lin.Cross(m.r2, normY))
            //),
            //J1: new Float6(-1, 0, -oneCrossR1.x, 1, 0, oneCrossR2.x),
            //J: new Float6(0, -1, -oneCrossR1.y, 0, 1, oneCrossR2.y),
            M_inv: M_inv,
            //bias: m.delta * m.beta / dt,
            bias: (new float2(math.dot(m.delta, normal_n), math.dot(m.delta, normal_t))) * m.beta / dt,
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
            //Float6 P_n = constraint.GetImpulse(lambdaAccum.y);

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
        //Float6 P = constraint.GetImpulse(v, ref lambdaAccum.y);

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

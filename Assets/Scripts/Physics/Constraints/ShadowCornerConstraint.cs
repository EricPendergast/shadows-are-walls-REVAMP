using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;
using UnityEngine;

using ContactId = Physics.Math.Geometry.ContactId;

using ShadowCornerManifold = ShadowEdgeGenerationSystem.ShadowCornerManifold;

public struct ShadowCornerConstraint : IConstraint {
    // TODO: Much of these fields don't need to be stored
    // The non opaque object
    public Entity e1 {get;}
    // The opaque object
    public Entity e2 {get;}
    public Entity e3 {get;}
    private Lambdas accum;
    public Lambdas GetAccumulatedLambdas() {
        return accum;
    }

    public int id {get;}

    Float9 M_inv;

    PenetrationConstraint<Float9> penConstraint;

    private enum ResolveMode {
        Depth,
        Breadth
    }

    public void Complete(ref ComponentDataFromEntity<Box> boxes, ref ComponentDataFromEntity<LightSource> lights) {

    }

    public struct Partial {
        public Entity e1;
        public Entity e2;
        public Entity e3;
        public ShapeType e1Type;
        public ShapeType e2Type;
        public ShapeType e3Type;
        Float9 jacobian;
    }

    public static void GetPartialConstraints(in ShadowCornerManifold m, ref NativeList<Partial> constraints) {
        Partial partialInit;
        partialInit.e1 = m.e1;
        partialInit.e2 = m.e2;
        partialInit.e3 = m.e3;

        Float9 J_n = new Float9();

        ResolveMode resolveMode;
        float delta;
        {
            float depth = Lin.IsFinite(m.p) ?
                math.dot(m.p - m.s, m.n) :
                -math.INFINITY;
        
            float2 breadthVec = m.p1 - m.p2;
        
            float breadth = Lin.IsFinite(breadthVec) ? -math.length(breadthVec) : -math.INFINITY;
        
            if (depth >= breadth) {
                delta = depth;
                resolveMode = ResolveMode.Depth;
            } else {
                delta = breadth;
                resolveMode = ResolveMode.Breadth;
            }
        }
        Debug.Assert(Lin.IsFinite(delta));

        if (resolveMode == ResolveMode.Depth) {
            // partial of delta wrt alpha1 = (-d_2 * n) * (d_1 cross d_2) * mag(p - x1)
            // partial of delta wrt alpha2 = (-d_1 * n) * (d_2 cross d_1) * mag(p - x2)
            // partial of delta wrt x_3 = -n
            // partial of delta wrt alpha3 = -(p-x_3) cross n
            
            //float2 d_1 = math.normalize(m. - x_1);
            //float2 d_2 = math.normalize(m.lightSource2 - x_2);
            //float2 p = m.lineOppositeCorner;
            J_n = new Float9(
                new float3(0, 0, (-math.dot(m.d2, m.n) * Lin.Cross(m.d2, m.d1) * math.length(m.p - m.x1))),
                new float3(0, 0, (-math.dot(m.d1, m.n) * Lin.Cross(m.d1, m.d2) * math.length(m.p - m.x2))),
                new float3(-m.n, -Lin.Cross(m.p - m.x3, m.n))
            );
        } else {
            J_n = new Float9(
                new float3(0, 0, math.dot(m.s - m.x1, m.n)/(math.lengthsq(m.d1)*math.lengthsq(m.n))),
                new float3(0, 0, math.dot(m.s - m.x2, m.n)/(math.lengthsq(m.d2)*math.lengthsq(m.n))),
                new float3(
                    (Lin.Cross(m.d1, m.n) + Lin.Cross(m.d2, m.n))*m.n,
                    Lin.Cross(m.p1 - m.x3, Lin.Cross(m.n, 1)) +
                        Lin.Cross(m.p2 - m.x3, Lin.Cross(m.n, 1)))
            );
        }
    }

    public void PreStep(ref Velocity v1, ref Velocity v2, ref Velocity v3, float dt, Lambdas prevLambdas) {
        accum = prevLambdas;

        if (CollisionSystem.accumulateImpulses) {
            Float9 P_n = penConstraint.GetImpulse(accum.n);

            ApplyImpulse(P_n, ref v1, ref v2, ref v3);
        }
    }

    public void ApplyImpulse(ref Velocity v1, ref Velocity v2, ref Velocity v3, float dt) {
        Float9 v = GetV(ref v1, ref v2, ref v3);

        Float9 P = penConstraint.GetImpulse(v, ref accum.n);

        ApplyImpulse(P, ref v1, ref v2, ref v3);
    }

    private static Float9 GetV(ref Velocity v1, ref Velocity v2, ref Velocity v3) {
        return new Float9(
            new float3(v1.vel, v1.angVel),
            new float3(v2.vel, v2.angVel),
            new float3(v3.vel, v3.angVel)
        );
    }

    private void ApplyImpulse(Float9 impulse, ref Velocity v1, ref Velocity v2, ref Velocity v3) {
        impulse = impulse.Mult(M_inv);

        v1.vel += impulse.v1.xy;
        v1.angVel += impulse.v1.z;

        v2.vel += impulse.v2.xy;
        v2.angVel += impulse.v2.z;

        v3.vel += impulse.v3.xy;
        v3.angVel += impulse.v3.z;
    }
}

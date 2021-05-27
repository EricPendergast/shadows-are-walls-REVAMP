using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;
using UnityEngine;

using ShadowCornerManifold = ShadowEdgeGenerationSystem.ShadowCornerManifold;

public struct ShadowCornerConstraint : IConstraint {
    // The owner of the first shadow edge
    public Entity e1 {get;}
    // The owner of the second shadow edge
    public Entity e2 {get;}
    // The owner of the shadow hitting object, or the third shadow edge
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

    public ShadowCornerConstraint(in Partial p, ComponentDataFromEntity<Mass> masses, float dt) {
        e1 = p.e1;
        e2 = p.e2;
        e3 = p.e3;

        Float9 M_inv = new Float9(masses[e1].M_inv, masses[e2].M_inv, masses[e3].M_inv);
        id = p.id;
        penConstraint = new PenetrationConstraint<Float9>(p.J_n, M_inv, p.bias/dt);
        this.M_inv = M_inv;

        accum = new Lambdas();
    }

    public struct Partial {
        public Entity e1;
        public Entity e2;
        public Entity e3;
        public int id;

        public Float9 J_n;
        public float bias;

        private static void HandleMount(in CornerCalculator.EdgeMount mount, ref float3 J_n_part, float2 x) {
            if (mount.castingShapeType == EdgeSourceType.Box) {
                float2 velMult = Lin.Cross(1, mount.point - x)/math.lengthsq(mount.point - x);
                float angVelMult = math.dot(mount.point - mount.shapeCenter, mount.point - x)/math.lengthsq(mount.point - x);
                J_n_part = new float3(
                    J_n_part.z * velMult,
                    J_n_part.z * angVelMult
                );
            }
        }

        // Use contact between three shadow edges
        public Partial(in Prototype p, in CornerCalculator.EdgeMount em1, in CornerCalculator.EdgeMount em2, in CornerCalculator.EdgeMount em3, in ShadowCornerManifold m) {
            e1 = em1.castingEntity;
            e2 = em2.castingEntity;
            e3 = em3.castingEntity;
            id = em1.id ^ em2.id ^ em3.id ^ 406325; // Arbitrary number
            J_n = p.J_n;
            bias = p.bias;

            HandleMount(in em1, ref J_n.v1, m.x1);
            HandleMount(in em2, ref J_n.v2, m.x2);
            HandleMount(in em3, ref J_n.v3, m.x3);
        }

        // Use contact between 2 shadow edges and a rigidbody
        public Partial(in Prototype p, in CornerCalculator.EdgeMount em1, in CornerCalculator.EdgeMount em2, Entity e3, in ShadowCornerManifold m) {
            e1 = em1.castingEntity;
            e2 = em2.castingEntity;
            this.e3 = e3;
            id = new int2(m.contactIdOnBox, em1.id ^ em2.id).GetHashCode() ^ 98034; // Arbitrary number
            J_n = p.J_n;
            bias = p.bias;

            HandleMount(in em1, ref J_n.v1, m.x1);
            HandleMount(in em2, ref J_n.v2, m.x2);
        }

        public struct Prototype {
            public float bias;
            public Float9 J_n;

            public Prototype(in ShadowCornerManifold m) {
                ResolveMode resolveMode;
                float delta;
                {
                    float depth = Lin.IsFinite(m.p) ?
                        math.dot(m.p - m.s, m.n) :
                        -math.INFINITY;
                
                    float2 breadthVec = m.p1 - m.p2;
                
                    //float breadth = Lin.IsFinite(breadthVec) ? -math.length(breadthVec) : -math.INFINITY;
                    float breadth = Lin.IsFinite(breadthVec) ? Lin.Cross(breadthVec, m.n) : -math.INFINITY;
                
                    // Resolve whichever quantity has smaller magnitude
                    if (math.abs(depth) <= math.abs(breadth)) {
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
                    
                    J_n = new Float9(
                        new float3(0, 0, (-math.dot(m.d2, m.n) * Lin.Cross(m.d2, m.d1) * math.length(m.p - m.x1))),
                        new float3(0, 0, (-math.dot(m.d1, m.n) * Lin.Cross(m.d1, m.d2) * math.length(m.p - m.x2))),
                        new float3(-m.n, -Lin.Cross(m.p - m.x3, m.n))
                    );
                } else {

                    float s_n_mag = math.dot(m.s - m.x3, m.n);
                    float2 s_n = m.n * s_n_mag;
                    float2 n_perp = Lin.Cross(m.n, -1);

                    float alpha_3_part_1 =
                        (Lin.Cross(n_perp, m.d1)*Lin.Cross(m.d1, n_perp)*s_n_mag -
                            Lin.Cross(m.d1, m.x3 + s_n - m.x1)*Lin.Cross(m.d1, m.n))/
                        math.lengthsq(Lin.Cross(n_perp, m.d1));

                    float alpha_3_part_2 =
                        (Lin.Cross(n_perp, m.d2)*Lin.Cross(m.d2, n_perp)*s_n_mag -
                            Lin.Cross(m.d2, m.x3 + s_n - m.x2)*Lin.Cross(m.d2, m.n))/
                        math.lengthsq(Lin.Cross(n_perp, m.d2));

                    J_n = new Float9(
                        new float3(0, 0, -math.dot(m.s - m.x1, m.n)/(math.lengthsq(math.dot(m.d1, m.n)))),
                        new float3(0, 0, math.dot(m.s - m.x2, m.n)/(math.lengthsq(math.dot(m.d2, m.n)))),

                        new float3(
                            (Lin.Cross(m.d1, m.n) - Lin.Cross(m.d2, m.n))*m.n,
                            -alpha_3_part_1 + alpha_3_part_2
                        )
                    ).Mult(-math.sign(delta));
                    delta = -math.abs(delta);
                }

                float beta = CollisionSystem.positionCorrection ? .1f : 0;
                float delta_slop = -.01f;

                bias = 0;

                if (delta < delta_slop) {
                    bias = beta * (delta - delta_slop);
                }
            }
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

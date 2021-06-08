using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;
using UnityEngine;

public struct ShadowCornerManifold {
    public float2 x1;
    public float2 d1;
    public float2 x2;
    public float2 d2;
    public float2 x3;
    public float2 s;

    public float2 p;
    public float2 p1;
    public float2 p2;

    public float2 n;
    // This is only used if shape number 3 is not a shadow edge
    public int contactIdOnBox;
}

public struct ThreeWayPenConstraint : IConstraint {
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

    public ThreeWayPenConstraint(in Partial p, ComponentDataFromEntity<Mass> masses, float dt) {
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

        private static void HandleMount(in ShadowCornerCalculator.EdgeMount mount, ref float3 J_n_part, float2 x, float2 d) {
            float2 c = mount.point;
            float2 xPrime = mount.shapeCenter;
            if (mount.castingShapeType == EdgeSourceType.Box) {
                float2 velMult = Lin.Cross(1, d) / math.dot(c - x, d);
                float angVelMult = math.dot(c - xPrime, c - x)/math.lengthsq(c - x);
                J_n_part = new float3(
                    J_n_part.z * velMult,
                    J_n_part.z * angVelMult
                );
            }
        }

        // Use contact between three shadow edges
        public Partial(in Prototype p, in ShadowCornerCalculator.EdgeMount em1, in ShadowCornerCalculator.EdgeMount em2, in ShadowCornerCalculator.EdgeMount em3, float2 d3, in ShadowCornerManifold m) {
            e1 = em1.castingEntity;
            e2 = em2.castingEntity;
            e3 = em3.castingEntity;
            id = em1.id ^ em2.id ^ em3.id ^ 406325; // Arbitrary number
            J_n = p.J_n;
            bias = p.bias;
        
            HandleMount(in em1, ref J_n.v1, m.x1, m.d1);
            HandleMount(in em2, ref J_n.v2, m.x2, m.d2);
            HandleMount(in em3, ref J_n.v3, m.x3, d3);
        }

        // Use contact between 2 shadow edges and a rigidbody
        public Partial(in Prototype p, in ShadowCornerCalculator.EdgeMount em1, in ShadowCornerCalculator.EdgeMount em2, Entity e3, in ShadowCornerManifold m) {
            e1 = em1.castingEntity;
            e2 = em2.castingEntity;
            this.e3 = e3;
            id = new int2(m.contactIdOnBox, em1.id ^ em2.id).GetHashCode() ^ 98034; // Arbitrary number
            J_n = p.J_n;
            bias = p.bias;

            HandleMount(in em1, ref J_n.v1, m.x1, m.d1);
            HandleMount(in em2, ref J_n.v2, m.x2, m.d2);
        }

        public struct Prototype {
            public float bias;
            public Float9 J_n;

            public Prototype(in ShadowCornerManifold m) : 
                this(m: in m,
                   beta: CollisionSystem.positionCorrection ? .1f : 0,
                   delta_slop: -.01f) {
            }
            public Prototype(in ShadowCornerManifold m, float beta, float delta_slop) {
                ResolveMode resolveMode;
                float delta;
                {
                    float depth = Lin.IsFinite(m.p) ?
                        math.dot(m.p - m.s, m.n) :
                        -math.INFINITY;
                    if (depth > 0) {
                        depth = -math.INFINITY;
                    }
                
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
                if (!math.isfinite(delta)) {
                    Debug.Assert(false, "ShadowCornerConstraint has infinite penetration");
                    J_n = new Float9(0,0,0);
                    bias = 0;
                    return;
                }

                if (resolveMode == ResolveMode.Depth) {
                    J_n = new Float9(
                        new float3(0, 0, 
                            (math.dot(m.d2, m.n) * Lin.Cross(m.d2, m.x1 - m.x2) /
                                math.lengthsq(Lin.Cross(m.d1, m.d2)))),
                        new float3(0, 0, 
                            (-math.dot(m.d1, m.n) * Lin.Cross(m.d1, m.x1 - m.x2) /
                                math.lengthsq(Lin.Cross(m.d1, m.d2)))),
                        new float3(-m.n, -Lin.Cross(m.p - m.x3, m.n))
                    );
                } else {
                    float2 n = m.n;
                    float2 x1 = m.x1;
                    float2 x2 = m.x2;
                    float2 x3 = m.x3;
                    float2 s = m.s;
                    float2 d1 = m.d1;
                    float2 d2 = m.d2;

                    float sigma = -1;
                    if (delta > 0) {
                        sigma = 1;
                        delta = -delta;
                    }

                    Debug.Assert(math.dot(m.p2 - m.p1, Lin.Cross(n, sigma)) <= 0);

                    float sMag = math.dot(m.s - m.x3, m.n);

                    float dAlpha1 = sigma * (sMag + math.dot(n, x3 - x1)) /
                        math.lengthsq(math.dot(m.d1, m.n));
                    float dAlpha2 = -sigma * (sMag + math.dot(n, x3 - x2)) /
                        math.lengthsq(math.dot(d2, n));

                    float dAlpha3 = sigma * (
                        -Lin.Cross(n, d1)*Lin.Cross(n*sMag - x1 + x3, d1) /
                            math.lengthsq(math.dot(d1, n)) +
                        Lin.Cross(n, d2)*Lin.Cross(n*sMag - x2 + x3, d2) /
                            math.lengthsq(math.dot(d2, n))
                    );

                    float2 dX3 = sigma * n * Lin.Cross(d2, d1) / (math.dot(d1, n) * math.dot(d2, n));

                    J_n = new Float9(
                        new float3(0, 0, dAlpha1),
                        new float3(0, 0, dAlpha2),
                        new float3(dX3, dAlpha3)
                    );
                }

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

        v1 += impulse.v1;

        v2 += impulse.v2;

        v3 += impulse.v3;

        if (e1 == e2) {
            v1 += impulse.v2;
            v2 += impulse.v1;
        }
        if (e1 == e3) {
            v1 += impulse.v3;
            v3 += impulse.v1;
        }
        if (e2 == e3) {
            v2 += impulse.v3;
            v3 += impulse.v2;
        }
    }
}

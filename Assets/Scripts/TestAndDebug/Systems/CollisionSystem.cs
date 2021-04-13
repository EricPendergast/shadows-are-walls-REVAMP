using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using Physics.Math;
using UnityEngine;

using ContactId = Physics.Math.Geometry.ContactId;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(GravitySystem))]
public class CollisionSystem : SystemBase {
    EntityQuery boxesQuery;
    static bool accumulateImpulses = true;
    static bool warmStarting = true;
    static bool positionCorrection = true;
    // In the future, this will be configurable on a per object basis
    static float globalFriction = .2f;

    private struct BoxBoxConstraint {
        public Entity box1;
        public Entity box2;
        public Lambdas accumulatedLambdas;
        public float2 normal;
        public float2 contact;
        public ContactId id;

        float3x3 M1_inv;
        float3x3 M2_inv;

        // Ideally, we would have a float6 J, but the library only goes up
        // to float4, so J is split into 2 pieces.
        float3 J1_n;
        float3 J2_n;

        float m_c_n;

        float3 J1_t;
        float3 J2_t;

        float m_c_t;

        public void PreStep(ref ComponentDataFromEntity<Box> boxes, float dt) {
            Box box1 = boxes[this.box1];
            Box box2 = boxes[this.box2];

            M1_inv = new float3x3(
                1/box1.mass, 0, 0,
                0, 1/box1.mass, 0,
                0, 0, 1/box1.inertia
            );
            M2_inv = new float3x3(
                1/box2.mass, 0, 0,
                0, 1/box2.mass, 0,
                0, 0, 1/box2.inertia
            );

            { // Normal precomputation
                J1_n = new float3(-normal, -Lin.Cross(contact-box1.pos, normal));
                J2_n = new float3(normal, Lin.Cross(contact-box2.pos, normal));

                m_c_n = 1 / (math.dot(math.mul(J1_n, M1_inv), J1_n) + math.dot(math.mul(J2_n, M2_inv), J2_n));
            }


            { // Tangent precomputation (friction)
                float2 tangent = Lin.Cross(normal, -1);

                J1_t = new float3(tangent, Lin.Cross(contact-box1.pos, tangent));
                J2_t = new float3(-tangent, -Lin.Cross(contact-box2.pos, tangent));
                
                m_c_t = 1 / (math.dot(math.mul(J1_t, M1_inv), J1_t) + math.dot(math.mul(J2_t, M2_inv), J2_t));
            }

            if (accumulateImpulses) {
                // Impulse
                float3 P_c1 = J1_n*accumulatedLambdas.n + J1_t*accumulatedLambdas.t;
                float3 P_c2 = J2_n*accumulatedLambdas.n + J2_t*accumulatedLambdas.t;

                // Delta velocity
                float3 dv1 = math.mul(M1_inv, P_c1);
                float3 dv2 = math.mul(M2_inv, P_c2);

                box1.vel += dv1.xy;
                box1.angVel += dv1.z;

                box2.vel += dv2.xy;
                box2.angVel += dv2.z;
            }

            boxes[this.box1] = box1;
            boxes[this.box2] = box2;
        }

        public void ApplyImpulse(ref ComponentDataFromEntity<Box> boxes, float dt) {
            Box box1 = boxes[this.box1];
            Box box2 = boxes[this.box2];

            float delta = -Geometry.GetOverlapOnAxis(box1.ToRect(), box2.ToRect(), normal);

            float lambda;
            {
                float3 v1 = new float3(box1.vel, box1.angVel);
                float3 v2 = new float3(box2.vel, box2.angVel);


                float beta = positionCorrection ? .1f : 0;
                float delta_slop = -.01f;

                float bias = 0;

                if (delta < delta_slop) {
                    bias = (beta/dt) * (delta - delta_slop);
                }

                lambda = -m_c_n * (math.dot(J1_n, v1) + math.dot(J2_n, v2) + bias);

                ClampLambda(ref lambda);

                // Impulse
                float3 P_c1 = J1_n*lambda;
                float3 P_c2 = J2_n*lambda;

                // Delta velocity
                float3 dv1 = math.mul(M1_inv, P_c1);
                float3 dv2 = math.mul(M2_inv, P_c2);

                box1.vel += dv1.xy;
                box1.angVel += dv1.z;

                box2.vel += dv2.xy;
                box2.angVel += dv2.z;

            }

            ////////////////////////////////
            /////////// Friction ///////////
            ////////////////////////////////

            {
                float3 v1 = new float3(box1.vel, box1.angVel);
                float3 v2 = new float3(box2.vel, box2.angVel);

                float lambda_t = -m_c_t * (math.dot(J1_t, v1) + math.dot(J2_t, v2));

                // Frictional coefficient
                float mu = globalFriction;

                ClampLambda_t(ref lambda_t, lambda,  mu);

                float3 P_t1 = J1_t*lambda_t;
                float3 P_t2 = J2_t*lambda_t;

                // Delta velocity
                float3 dv1 = math.mul(M1_inv, P_t1);
                float3 dv2 = math.mul(M2_inv, P_t2);

                box1.vel += dv1.xy;
                box1.angVel += dv1.z;

                box2.vel += dv2.xy;
                box2.angVel += dv2.z;
            }

            // NOTE: This is not thread safe
            boxes[this.box1] = box1;
            boxes[this.box2] = box2;
        }


        private void ClampLambda(ref float lambda) {
            if (accumulateImpulses) {
                float oldAccumulated = accumulatedLambdas.n;
                accumulatedLambdas.n = math.max(accumulatedLambdas.n + lambda, 0);
                lambda = accumulatedLambdas.n - oldAccumulated;
            } else {
                lambda = math.max(lambda, 0);
            }
        }

        private void ClampLambda_t(ref float lambda_t, float lambda, float frictionCoefficient) {
            if (accumulateImpulses) {
                float old_tAccumulated = accumulatedLambdas.t;
                accumulatedLambdas.t = math.clamp(accumulatedLambdas.t + lambda_t, -accumulatedLambdas.n*frictionCoefficient, accumulatedLambdas.n*frictionCoefficient);
                lambda_t = accumulatedLambdas.t - old_tAccumulated;
            } else {
                lambda_t = math.clamp(lambda_t, -lambda*frictionCoefficient, lambda*frictionCoefficient);
            }
        }
    }

    private NativeList<Entity> boxEntities;
    private NativeList<BoxBoxConstraint> boxBoxConstraints;
    // Maps from contact id to the accumulated lambda of that contact last
    // frame. Used for warm starting.
    private struct Lambdas {
        public float n;
        public float t;
    }
    private NativeHashMap<ContactId, Lambdas> prevLambdas;

    protected override void OnCreate() {
        boxEntities = new NativeList<Entity>(100, Allocator.Persistent);
        boxBoxConstraints = new NativeList<BoxBoxConstraint>(100, Allocator.Persistent);
        prevLambdas = new NativeHashMap<ContactId, Lambdas>(100, Allocator.Persistent);
    }

    protected override void OnDestroy() {
        boxEntities.Dispose();
        boxBoxConstraints.Dispose();
        prevLambdas.Dispose();
    }

    protected override void OnUpdate() {
        var boxEntities = this.boxEntities;
        boxEntities.Clear();
        boxEntities.Length = boxesQuery.CalculateEntityCount();
        boxBoxConstraints.Clear();

        var boxes = GetComponentDataFromEntity<Box>(false);

        Entities
            .WithName("StoreBoxes")
            .WithAll<Box>()
            .WithStoreEntityQueryInField(ref boxesQuery)
            .ForEach((int entityInQueryIndex, ref Entity e) => {
                boxEntities[entityInQueryIndex] = e;
            }).Run();

        // Creating constraints for each contact point between two boxes
        for (int i = 0; i < boxEntities.Length; i++ ) {
            for (int j = i+1; j < boxEntities.Length; j++ ) {
                Entity box1 = boxEntities[i];
                Entity box2 = boxEntities[j];

                var manifoldNullable = Geometry.GetIntersectData(
                    boxes[box1].ToRect(),
                    boxes[box2].ToRect()
                );

                if (manifoldNullable is Geometry.Manifold manifold) {

                    var constraint = 
                        new BoxBoxConstraint{
                            box1=box1, box2=box2, 
                            normal=manifold.normal, 
                            contact=manifold.contact1.point,
                            id=manifold.contact1.id,
                        };

                    boxBoxConstraints.Add(constraint);


                    if (manifold.contact2 is Geometry.Contact contact) {
                        constraint.contact = contact.point;
                        constraint.id = contact.id;
                        boxBoxConstraints.Add(constraint);
                        Debug.Assert(!manifold.contact1.id.Equals(contact.id), "Duplicate contact ids within the same manifold");
                    }
                }
            }
        }

        float dt = Time.DeltaTime;

        for (int j = 0; j < boxBoxConstraints.Length; j++) {
            var c = boxBoxConstraints[j];

            if (warmStarting && prevLambdas.TryGetValue(c.id, out Lambdas lambdas)) {
                c.accumulatedLambdas = lambdas;
            } else {
                c.accumulatedLambdas = new Lambdas();
            }

            c.PreStep(ref boxes, dt);

            // TODO: Non readonly structs are EVIL
            boxBoxConstraints[j] = c;
        }

        for (int i = 0; i < 10; i++) {
            for (int j = 0; j < boxBoxConstraints.Length; j++) {
                var c = boxBoxConstraints[j];

                c.ApplyImpulse(ref boxes, dt);

                // TODO: Non readonly structs are EVIL
                boxBoxConstraints[j] = c;
            }
        }

        prevLambdas.Clear();
        foreach (var constraint in boxBoxConstraints) {
            // TODO: This assert actually fails naturally sometimes. It's
            // because sometimes contact ids can refer to the corner of a
            // shape, which may be intersecting multiple other shapes, thus
            // giving duplicate contact ids. Having arbiters would fix this.
            // But I think its not a big deal.
            Debug.Assert(!prevLambdas.ContainsKey(constraint.id), "Duplicate contact id: " + constraint.id.ToString());
            prevLambdas[constraint.id] = constraint.accumulatedLambdas;
        }
    }

    public struct DebugContactInfo {
        public float2 normal;
        public float2 contact;
        public ContactId id;
    }

    public IEnumerable<DebugContactInfo> GetContactsForDebug() {
        foreach (var constraint in boxBoxConstraints) {
            yield return new DebugContactInfo{normal = constraint.normal, contact = constraint.contact, id = constraint.id};
        }
    }
}

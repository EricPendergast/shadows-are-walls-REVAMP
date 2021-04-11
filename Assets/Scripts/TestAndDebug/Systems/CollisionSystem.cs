using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using Physics.Math;
using UnityEngine;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(GravitySystem))]
public class CollisionSystem : SystemBase {
    EntityQuery boxesQuery;
    static bool accumulateImpulses = true;
    static bool warmStarting = false;
    static bool positionCorrection = true;
    // In the future, this will be configurable on a per object basis
    static float globalFriction = .5f;

    private struct BoxBoxConstraint {
        public Entity box1;
        public Entity box2;
        public float lambdaAccumulated;
        public float lambda_tAccumulated;
        public float2 normal;
        public float2 contact;
        public int id;

        public void PreStep(ref ComponentDataFromEntity<Box> boxes, float dt) {
            if (warmStarting) {
                // TODO: Apply lambdaAccumulated and lambda_tAccumulated
            }
        }

        public void ApplyImpulse(ref ComponentDataFromEntity<Box> boxes, float dt) {
            Box box1 = boxes[this.box1];
            Box box2 = boxes[this.box2];

            float delta = -Geometry.GetOverlapOnAxis(box1.ToRect(), box2.ToRect(), normal);
            // If not intersecting, and no impulse has been applied yet, do nothing
            //if (delta > 0) {
            //    return;
            //}

            float3x3 M1_inv = new float3x3(
                1/box1.mass, 0, 0,
                0, 1/box1.mass, 0,
                0, 0, 1/box1.inertia
            );
            float3x3 M2_inv = new float3x3(
                1/box2.mass, 0, 0,
                0, 1/box2.mass, 0,
                0, 0, 1/box2.inertia
            );

            float lambda;
            {
                // Ideally, we would have a float6 J, but the library only goes up
                // to float4, so J is split into 2 pieces.
                float3 J1 = new float3(-normal, -Lin.Cross(contact-box1.pos, normal));
                float3 J2 = new float3(normal, Lin.Cross(contact-box2.pos, normal));


                float3 v1 = new float3(box1.vel, box1.angVel);
                float3 v2 = new float3(box2.vel, box2.angVel);

                float m_c = 1 / (math.dot(math.mul(J1, M1_inv), J1) + math.dot(math.mul(J2, M2_inv), J2));

                float beta = positionCorrection ? .2f : 0;
                float delta_slop = -.01f;

                float bias = 0;

                if (delta < delta_slop) {
                    bias = (beta/dt) * (delta - delta_slop);
                }

                lambda = -m_c * (math.dot(J1, v1) + math.dot(J2, v2) + bias);

                ClampLambda(ref lambda);

                // Impulse
                float3 P_c1 = J1*lambda;
                float3 P_c2 = J2*lambda;

                // Delta velocity
                float3 dv1 = math.mul(M1_inv, P_c1);
                float3 dv2 = math.mul(M2_inv, P_c2);

                box1.vel += dv1.xy;
                box1.angVel += dv1.z;

                box2.vel += dv2.xy;
                box2.angVel += dv2.z;

                ///////////////////////
                /// From box2d lite ///
                ///////////////////////

                //float2 r1 = contact - box1.pos;
                //float2 r2 = contact - box2.pos;
                //
                //float invMass1 = box1.mass == float.MaxValue ? 0 : 1/box1.mass;
                //float invMass2 = box2.mass == float.MaxValue ? 0 : 1/box2.mass;
                //float invI1 = box1.mass == float.MaxValue ? 0 : 1/box1.inertia;
                //float invI2 = box2.mass == float.MaxValue ? 0 : 1/box2.inertia;
                //
                //// Relative velocity at contact
                //float2 dv = box2.vel + Lin.Cross(box2.angVel, r2) - box1.vel - Lin.Cross(box1.angVel, r1);
                //
                //// Compute normal impulse
                //float vn = math.dot(dv, normal);
                //
                //    float rn1 = math.dot(r1, normal);
                //    float rn2 = math.dot(r2, normal);
                //    float kNormal = (invMass1) + (invMass2);
                //    kNormal += (invI1) * (math.dot(r1, r1) - rn1 * rn1) + (invI2) * (math.dot(r2, r2) - rn2 * rn2);
                //    float massNormal = 1.0f / kNormal;
                //
                //float dPn = massNormal * (-vn);
                //
                //dPn = math.max(dPn, 0.0f);
                //
                //Debug.Log("dPn = " + dPn + " dv = " + dv.x + " " + dv.y + " mass normal = " + massNormal);
                //
                //// Apply contact impulse
                //float2 Pn = dPn * normal;
                //
                //box1.vel -= invMass1 * Pn;
                //box1.angVel -= invI1 * Lin.Cross(r1, Pn);
                //
                //box2.vel += invMass2 * Pn;
                //box2.angVel += invI2 * Lin.Cross(r2, Pn);
            }

            ////////////////////////////////
            /////////// Friction ///////////
            ////////////////////////////////

            {
                float2 tangent = Lin.Cross(normal, -1);

                float3 J1_t = new float3(tangent, Lin.Cross(contact-box1.pos, tangent));
                float3 J2_t = new float3(-tangent, -Lin.Cross(contact-box2.pos, tangent));
                
                float m_t = 1 / (math.dot(math.mul(J1_t, M1_inv), J1_t) + math.dot(math.mul(J2_t, M2_inv), J2_t));

                float3 v1 = new float3(box1.vel, box1.angVel);
                float3 v2 = new float3(box2.vel, box2.angVel);

                float lambda_t = -m_t * (math.dot(J1_t, v1) + math.dot(J2_t, v2));

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
                float oldAccumulated = lambdaAccumulated;
                lambdaAccumulated = math.max(lambdaAccumulated + lambda, 0);
                lambda = lambdaAccumulated - oldAccumulated;
            } else {
                lambda = math.max(lambda, 0);
            }
        }

        private void ClampLambda_t(ref float lambda_t, float lambda, float frictionCoefficient) {
            if (accumulateImpulses) {
                float old_tAccumulated = lambda_tAccumulated;
                lambda_tAccumulated = math.clamp(lambda_tAccumulated + lambda_t, -lambdaAccumulated*frictionCoefficient, lambdaAccumulated*frictionCoefficient);
                lambda_t = lambda_tAccumulated - old_tAccumulated;
            } else {
                lambda_t = math.clamp(lambda_t, -lambda*frictionCoefficient, lambda*frictionCoefficient);
            }
        }
    }

    private NativeList<Entity> boxEntities;
    private NativeList<BoxBoxConstraint> boxBoxConstraints;
    // Maps from contact id to the accumulated lambda of that contact last
    // frame. Used for warm starting.
    private NativeHashMap<int, float> prevLambdas;

    protected override void OnCreate() {
        boxEntities = new NativeList<Entity>(100, Allocator.Persistent);
        boxBoxConstraints = new NativeList<BoxBoxConstraint>(100, Allocator.Persistent);
        prevLambdas = new NativeHashMap<int, float>(100, Allocator.Persistent);
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
                        Debug.Assert(manifold.contact1.id != contact.id, "Duplicate contact ids within the same manifold");
                    }
                }
            }
        }

        float dt = Time.DeltaTime;

        for (int j = 0; j < boxBoxConstraints.Length; j++) {
            var c = boxBoxConstraints[j];

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
            Debug.Assert(!prevLambdas.ContainsKey(constraint.id), "Duplicate contact id");
            prevLambdas[constraint.id] = constraint.lambdaAccumulated;
        }
    }

    public struct DebugContactInfo {
        public float2 normal;
        public float2 contact;
        public int id;
    }

    public IEnumerable<DebugContactInfo> GetContactsForDebug() {
        foreach (var constraint in boxBoxConstraints) {
            yield return new DebugContactInfo{normal = constraint.normal, contact = constraint.contact, id = constraint.id};
        }
    }
}

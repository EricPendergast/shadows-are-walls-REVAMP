using Unity.Entities;
using Unity.Transforms;
using Unity.Collections;
using Unity.Mathematics;

using Physics.Math;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(GravitySystem))]
public class CollisionSystem : SystemBase {
    EntityQuery boxesQuery;

    private struct BoxBoxConstraint {
        public Entity box1;
        public Entity box2;
        public float lambdaAccumulated;
        public float lambda_tAccumulated;
        public float2 normal;
        public float2 contact;


        public void ApplyImpulse(ref ComponentDataFromEntity<Box> boxes, float dt) {
            Box box1 = boxes[this.box1];
            Box box2 = boxes[this.box2];

            // Ideally, we would have a float6 J, but the library only goes up
            // to float4, so J is split into 2 pieces.
            float3 J1 = new float3(-normal, -Lin.Cross(contact-box1.pos, normal));
            float3 J2 = new float3(normal, Lin.Cross(contact-box2.pos, normal));

            float2 tangent = Lin.Cross(normal, -1);

            float3 J1_t = new float3(tangent, Lin.Cross(contact-box1.pos, tangent));
            float3 J2_t = new float3(-tangent, -Lin.Cross(contact-box2.pos, tangent));
            
            // Similar situation with M
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

            float3 v1 = new float3(box1.vel, box1.angVel);
            float3 v2 = new float3(box2.vel, box2.angVel);

            float m_c = 1 / (math.dot(math.mul(J1, M1_inv), J1) + math.dot(math.mul(J2, M2_inv), J2));
            float m_t = 1 / (math.dot(math.mul(J1_t, M1_inv), J1_t) + math.dot(math.mul(J2_t, M2_inv), J2_t));

            float beta = .3f;
            float delta_slop = -.05f;
            float delta = -Geometry.GetOverlapOnAxis(box1.ToRect(), box2.ToRect(), normal);

            float bias = 0;

            if (delta < delta_slop) {
                bias = beta/dt * (delta - delta_slop);
            }

            float lambda = -m_c * (math.dot(J1, v1) + math.dot(J2, v2) + bias);
            float lambda_t = -m_t * (math.dot(J1_t, v1) + math.dot(J2_t, v2));

            // Frictional coefficient
            float mu = .5f;
            ClampLambdas(ref lambda, ref lambda_t, mu);

            // Impulse
            float3 P_c1 = J1*lambda + J1_t*lambda_t;
            float3 P_c2 = J2*lambda + J2_t*lambda_t;

            // Delta velocity
            float3 dv1 = math.mul(M1_inv, P_c1);
            float3 dv2 = math.mul(M2_inv, P_c2);

            box1.vel += dv1.xy;
            box1.angVel += dv1.z;

            box2.vel += dv2.xy;
            box2.angVel += dv2.z;

            // NOTE: This is not thread safe
            boxes[this.box1] = box1;
            boxes[this.box2] = box2;
        }

        private void ClampLambdas(ref float lambda, ref float lambda_t, float frictionCoefficient) {
            float oldAccumulated = lambdaAccumulated;
            lambdaAccumulated = math.max(lambdaAccumulated + lambda, 0);
            lambda = lambdaAccumulated - oldAccumulated;

            float old_tAccumulated = lambda_tAccumulated;
            lambda_tAccumulated = math.clamp(lambda_tAccumulated + lambda_t, -lambdaAccumulated*frictionCoefficient, lambdaAccumulated*frictionCoefficient);
            lambda_t = lambda_tAccumulated - old_tAccumulated;
        }
    }

    private NativeList<Entity> boxEntities;
    private NativeList<BoxBoxConstraint> boxBoxConstraints;

    protected override void OnCreate() {
        boxEntities = new NativeList<Entity>(100, Allocator.Persistent);
        boxBoxConstraints = new NativeList<BoxBoxConstraint>(100, Allocator.Persistent);
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
                            contact=manifold.contact1
                        };

                    boxBoxConstraints.Add(constraint);

                    if (manifold.contact2 is float2 contact) {
                        constraint.contact = contact;
                        boxBoxConstraints.Add(constraint);
                    }
                }
            }
        }

        float dt = Time.DeltaTime;

        for (int i = 0; i < 4; i++) {
            foreach (var constraint in boxBoxConstraints) {
                constraint.ApplyImpulse(ref boxes, dt);
            }
        }
    }
}

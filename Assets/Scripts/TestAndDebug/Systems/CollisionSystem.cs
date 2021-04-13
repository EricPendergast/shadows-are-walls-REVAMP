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
    public static bool accumulateImpulses = true;
    public static bool warmStarting = true;
    public static bool positionCorrection = true;
    // In the future, this will be configurable on a per object basis
    public static float globalFriction = .2f;

    private NativeList<Entity> boxEntities;
    private NativeList<BoxBoxConstraint> boxBoxConstraints;
    // Maps from contact id to the accumulated lambda of that contact last
    // frame. Used for warm starting.
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

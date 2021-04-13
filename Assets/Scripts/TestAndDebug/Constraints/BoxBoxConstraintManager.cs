using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using UnityEngine;

using Physics.Math;

using ContactId = Physics.Math.Geometry.ContactId;

public class BoxBoxConstraintManager {
    private NativeList<BoxBoxConstraint> boxBoxConstraints;
    // Maps from contact id to the accumulated lambda of that contact last
    // frame. Used for warm starting.
    private NativeHashMap<ContactId, Lambdas> prevLambdas;

    public BoxBoxConstraintManager() {
        prevLambdas = new NativeHashMap<ContactId, Lambdas>(100, Allocator.Persistent);
        boxBoxConstraints = new NativeList<BoxBoxConstraint>(100, Allocator.Persistent);
    }

    public void FindConstraints(ref ComponentDataFromEntity<Box> boxes, NativeList<Entity> boxEntities) {

        boxBoxConstraints.Clear();

        for (int i = 0; i < boxEntities.Length; i++ ) {
            for (int j = i+1; j < boxEntities.Length; j++ ) {
                Entity box1 = boxEntities[i];
                Entity box2 = boxEntities[j];

                var manifoldNullable = Geometry.GetIntersectData(
                    boxes[box1].ToRect(),
                    boxes[box2].ToRect()
                );

                if (manifoldNullable is Geometry.Manifold manifold) {

                    boxBoxConstraints.Add(new BoxBoxConstraint(
                        box1, box2,
                        manifold.normal,
                        manifold.contact1
                    ));


                    if (manifold.contact2 is Geometry.Contact contact) {
                        boxBoxConstraints.Add(new BoxBoxConstraint(
                            box1, box2,
                            manifold.normal,
                            contact
                        ));

                        Debug.Assert(!manifold.contact1.id.Equals(contact.id), "Duplicate contact ids within the same manifold");
                    }
                }
            }
        }
    }

    public void PreStep(ref ComponentDataFromEntity<Box> boxes, float dt) {
        for (int j = 0; j < boxBoxConstraints.Length; j++) {
            var c = boxBoxConstraints[j];

            Lambdas lambdas;
            if (CollisionSystem.warmStarting && prevLambdas.TryGetValue(c.id, out var l)) {
                lambdas = l;
            } else {
                lambdas = new Lambdas();
            }

            c.PreStep(ref boxes, dt, lambdas);

            // TODO: Non readonly structs are EVIL
            boxBoxConstraints[j] = c;
        }
    }

    public void ApplyImpulse(ref ComponentDataFromEntity<Box> boxes, float dt) {

        for (int j = 0; j < boxBoxConstraints.Length; j++) {
            var c = boxBoxConstraints[j];

            c.ApplyImpulse(ref boxes, dt);

            // TODO: Non readonly structs are EVIL
            boxBoxConstraints[j] = c;
        }
    }

    public void StoreLambdas() {
        prevLambdas.Clear();
        foreach (var constraint in boxBoxConstraints) {
            // TODO: This assert actually fails naturally sometimes. It's
            // because sometimes contact ids can refer to the corner of a
            // shape, which may be intersecting multiple other shapes, thus
            // giving duplicate contact ids. Having arbiters would fix this.
            // But I think its not a big deal.
            Debug.Assert(!prevLambdas.ContainsKey(constraint.id), "Duplicate contact id: " + constraint.id.ToString());
            prevLambdas[constraint.id] = constraint.GetAccumulatedLambdas();
        }
    }

    public IEnumerable<CollisionSystem.DebugContactInfo> GetContactsForDebug() {
        foreach (var constraint in boxBoxConstraints) {
            yield return new CollisionSystem.DebugContactInfo{normal = constraint.normal, contact = constraint.contact, id = constraint.id};
        }
    }

    public void Destroy() {
        prevLambdas.Dispose();
        boxBoxConstraints.Dispose();
    }
}

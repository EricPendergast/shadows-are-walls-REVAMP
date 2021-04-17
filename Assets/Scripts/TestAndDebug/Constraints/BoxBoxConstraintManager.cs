using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using UnityEngine;

using Physics.Math;

using ContactId = Physics.Math.Geometry.ContactId;

public class BoxBoxConstraintManager : BoxBoxConstraintManagerGeneric {
    public ComponentDataFromEntity<Box> boxes;

    protected override Geometry.Manifold? GetManifold(Entity box1, Entity box2) {
        return Geometry.GetIntersectData(
            boxes[box1].ToRect(),
            boxes[box2].ToRect()
        );
    }

    protected override BoxBoxConstraint GetConstraint(Entity box1, Entity box2, Geometry.Manifold manifold, bool useContact1) {
        return new BoxBoxConstraint(
            box1, box2,
            manifold.normal,
            useContact1 ? manifold.contact1 : (Geometry.Contact)manifold.contact2
        );
    }

    protected override void ApplyImpulse(ref BoxBoxConstraint constraint, float dt) {
        constraint.ApplyImpulse(ref boxes, dt);
    }

    protected override void PreStep(ref BoxBoxConstraint constraint, float dt, Lambdas lambdas) {
        constraint.PreStep(ref boxes, dt, lambdas);
    }
}

public abstract class BoxBoxConstraintManagerGeneric {
    private NativeList<BoxBoxConstraint> boxBoxConstraints;
    // Maps from contact id to the accumulated lambda of that contact last
    // frame. Used for warm starting.
    private NativeHashMap<ContactId, Lambdas> prevLambdas;

    public BoxBoxConstraintManagerGeneric() {
        prevLambdas = new NativeHashMap<ContactId, Lambdas>(100, Allocator.Persistent);
        boxBoxConstraints = new NativeList<BoxBoxConstraint>(100, Allocator.Persistent);
    }

    public void FindConstraints(NativeList<Entity> boxEntities) {

        boxBoxConstraints.Clear();

        for (int i = 0; i < boxEntities.Length; i++ ) {
            for (int j = i+1; j < boxEntities.Length; j++ ) {
                Entity box1 = boxEntities[i];
                Entity box2 = boxEntities[j];

                var manifoldNullable = GetManifold(box1, box2);

                if (manifoldNullable is Geometry.Manifold manifold) {

                    boxBoxConstraints.Add(GetConstraint(box1, box2, manifold, true));

                    if (manifold.contact2 is Geometry.Contact contact) {

                        boxBoxConstraints.Add(GetConstraint(box1, box2, manifold, false));
                        Debug.Assert(!manifold.contact1.id.Equals(contact.id), "Duplicate contact ids within the same manifold");
                    }
                }
            }
        }
    }

    protected abstract Geometry.Manifold? GetManifold(Entity box1, Entity box2);
    protected abstract BoxBoxConstraint GetConstraint(Entity box1, Entity box2, Geometry.Manifold manifold, bool useContact1);
    protected abstract void ApplyImpulse(ref BoxBoxConstraint constraint, float dt);
    protected abstract void PreStep(ref BoxBoxConstraint constraint, float dt, Lambdas lambdas);

    public void PreSteps(float dt) {
        for (int j = 0; j < boxBoxConstraints.Length; j++) {
            var c = boxBoxConstraints[j];

            Lambdas lambdas;
            if (CollisionSystem.warmStarting && prevLambdas.TryGetValue(c.id, out var l)) {
                lambdas = l;
            } else {
                lambdas = new Lambdas();
            }

            PreStep(ref c, dt, lambdas);

            // TODO: Non readonly structs are EVIL
            boxBoxConstraints[j] = c;
        }
    }

    public void ApplyImpulses(float dt) {

        for (int j = 0; j < boxBoxConstraints.Length; j++) {
            var c = boxBoxConstraints[j];

            ApplyImpulse(ref c, dt);

            // TODO: Non readonly structs are EVIL
            boxBoxConstraints[j] = c;
        }
    }

    public void PostSteps() {
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

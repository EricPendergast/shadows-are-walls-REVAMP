using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using UnityEngine;

using Physics.Math;

using ContactId = Physics.Math.Geometry.ContactId;

public interface Constraint {
    public ContactId id {get;}
    public float2 normal {get;}
    public float2 contact {get;}
    public Lambdas GetAccumulatedLambdas();
}

public struct BoxBoxConstraintHelper : ConstraintManagerHelper<BoxBoxConstraint> {
    public ComponentDataFromEntity<Box> boxes;
    public ComponentDataFromEntity<Velocity> boxVels;
    public NativeList<Entity> boxEntities;

    private Geometry.Manifold? GetManifold(Entity box1, Entity box2) {
        return Geometry.GetIntersectData(
            boxes[box1].ToRect(),
            boxes[box2].ToRect()
        );
    }

    private BoxBoxConstraint GetConstraint(Entity box1, Entity box2, Geometry.Manifold manifold, bool useContact1) {
        return new BoxBoxConstraint(
            box1, box2,
            manifold,
            useContact1
        );
    }

    public void ApplyImpulse(ref BoxBoxConstraint constraint, float dt) {
        var v1 = boxVels[constraint.box1];
        var v2 = boxVels[constraint.box2];

        constraint.ApplyImpulse(ref v1, ref v2, dt);

        boxVels[constraint.box1] = v1;
        boxVels[constraint.box2] = v2;
    }

    public void PreStep(ref BoxBoxConstraint constraint, float dt, Lambdas lambdas) {
        var v1 = boxVels[constraint.box1];
        var v2 = boxVels[constraint.box2];

        constraint.PreStep(boxes[constraint.box1], boxes[constraint.box2], ref v1, ref v2, dt, lambdas);

        boxVels[constraint.box1] = v1;
        boxVels[constraint.box2] = v2;
    }

    public void FillWithConstraints(NativeList<BoxBoxConstraint> constraints) {
        for (int i = 0; i < boxEntities.Length; i++ ) {
            for (int j = i+1; j < boxEntities.Length; j++ ) {
                Entity box1 = boxEntities[i];
                Entity box2 = boxEntities[j];

                var manifoldNullable = GetManifold(box1, box2);

                if (manifoldNullable is Geometry.Manifold manifold) {

                    constraints.Add(GetConstraint(box1, box2, manifold, true));

                    if (manifold.contact2 is Geometry.Contact contact) {

                        constraints.Add(GetConstraint(box1, box2, manifold, false));
                        Debug.Assert(!manifold.contact1.id.Equals(contact.id), "Duplicate contact ids within the same manifold");
                    }
                }
            }
        }
    }
}

public interface ConstraintManagerHelper<C> where C : struct, Constraint {
     void ApplyImpulse(ref C constraint, float dt);
     void PreStep(ref C constraint, float dt, Lambdas lambdas);
     void FillWithConstraints(NativeList<C> constraints);
}

public class ConstraintManager<H, C> 
        where C : struct, Constraint
        where H : struct, ConstraintManagerHelper<C> {

    private NativeList<C> constraints;
    private WarmStartManager warmStart;
    public H helper;

    public ConstraintManager() {
        constraints = new NativeList<C>(100, Allocator.Persistent);
        helper = default(H);
        warmStart = new WarmStartManager();
    }

    public void FindConstraints(NativeList<Entity> boxEntities) {
        constraints.Clear();

        helper.FillWithConstraints(constraints);
    }

    public void PreSteps(float dt) {
        for (int j = 0; j < constraints.Length; j++) {
            var c = constraints[j];

            helper.PreStep(ref c, dt, warmStart.GetLambdas(c.id));

            // TODO: Non readonly structs are EVIL
            constraints[j] = c;
        }
    }

    public void ApplyImpulses(float dt) {

        for (int j = 0; j < constraints.Length; j++) {
            var c = constraints[j];

            helper.ApplyImpulse(ref c, dt);

            // TODO: Non readonly structs are EVIL
            constraints[j] = c;
        }
    }

    public void PostSteps() {
        warmStart.SaveLambdas(constraints);
    }

    public IEnumerable<CollisionSystem.DebugContactInfo> GetContactsForDebug() {
        foreach (var constraint in constraints) {
            yield return new CollisionSystem.DebugContactInfo{normal = constraint.normal, contact = constraint.contact, id = constraint.id};
        }
    }

    public void Dispose() {
        warmStart.Dispose();
        constraints.Dispose();
    }

    public class WarmStartManager {
        // Maps from contact id to the accumulated lambda of that contact last
        // frame. Used for warm starting.
        private NativeHashMap<ContactId, Lambdas> prevLambdas;

        public WarmStartManager() {
            prevLambdas = new NativeHashMap<ContactId, Lambdas>(100, Allocator.Persistent);
        }

        public Lambdas GetLambdas(Geometry.ContactId id) {
            Lambdas lambdas;
            if (CollisionSystem.warmStarting && prevLambdas.TryGetValue(id, out var l)) {
                lambdas = l;
            } else {
                lambdas = new Lambdas();
            }
            return lambdas;
        }

        public void SaveLambdas(NativeArray<C> constraints) {
            prevLambdas.Clear();
            foreach (var constraint in constraints) {
                // TODO: This assert actually fails naturally sometimes. It's
                // because sometimes contact ids can refer to the corner of a
                // shape, which may be intersecting multiple other shapes, thus
                // giving duplicate contact ids. Having arbiters would fix this.
                // But I think its not a big deal.
                Debug.Assert(!prevLambdas.ContainsKey(constraint.id), "Duplicate contact id: " + constraint.id.ToString());
                prevLambdas[constraint.id] = constraint.GetAccumulatedLambdas();
            }
        }

        public void Dispose() {
            prevLambdas.Dispose();
        }
    }
}

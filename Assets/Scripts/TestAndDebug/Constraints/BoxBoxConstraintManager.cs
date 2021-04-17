using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using UnityEngine;

using Physics.Math;

using ContactId = Physics.Math.Geometry.ContactId;


public interface ConstraintManagerHelper {
     Geometry.Manifold? GetManifold(Entity box1, Entity box2);
     BoxBoxConstraint GetConstraint(Entity box1, Entity box2, Geometry.Manifold manifold, bool useContact1);
     void ApplyImpulse(ref BoxBoxConstraint constraint, float dt);
     void PreStep(ref BoxBoxConstraint constraint, float dt, Lambdas lambdas);
}

public class BoxBoxConstraintManager : BoxBoxConstraintManagerGeneric<BoxBoxConstraintHelper> {
    public ComponentDataFromEntity<Box> boxes {
        set => helper.boxes = value;
    }
    public ComponentDataFromEntity<Velocity> boxVels {
        set => helper.boxVels = value;
    }
    public BoxBoxConstraintManager() : base(default(BoxBoxConstraintHelper)) {

    }
}

public struct BoxBoxConstraintHelper : ConstraintManagerHelper {
    public ComponentDataFromEntity<Box> boxes;
    public ComponentDataFromEntity<Velocity> boxVels;

    public Geometry.Manifold? GetManifold(Entity box1, Entity box2) {
        return Geometry.GetIntersectData(
            boxes[box1].ToRect(),
            boxes[box2].ToRect()
        );
    }

    public BoxBoxConstraint GetConstraint(Entity box1, Entity box2, Geometry.Manifold manifold, bool useContact1) {
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
}

public abstract class BoxBoxConstraintManagerGeneric<H> 
        where H : struct, ConstraintManagerHelper {
    private NativeList<BoxBoxConstraint> boxBoxConstraints;
    // Maps from contact id to the accumulated lambda of that contact last
    // frame. Used for warm starting.
    private NativeHashMap<ContactId, Lambdas> prevLambdas;

    protected H helper;

    public BoxBoxConstraintManagerGeneric(H helper) {
        prevLambdas = new NativeHashMap<ContactId, Lambdas>(100, Allocator.Persistent);
        boxBoxConstraints = new NativeList<BoxBoxConstraint>(100, Allocator.Persistent);
        this.helper = helper;
    }

    public void FindConstraints(NativeList<Entity> boxEntities) {

        boxBoxConstraints.Clear();

        for (int i = 0; i < boxEntities.Length; i++ ) {
            for (int j = i+1; j < boxEntities.Length; j++ ) {
                Entity box1 = boxEntities[i];
                Entity box2 = boxEntities[j];

                var manifoldNullable = helper.GetManifold(box1, box2);

                if (manifoldNullable is Geometry.Manifold manifold) {

                    boxBoxConstraints.Add(helper.GetConstraint(box1, box2, manifold, true));

                    if (manifold.contact2 is Geometry.Contact contact) {

                        boxBoxConstraints.Add(helper.GetConstraint(box1, box2, manifold, false));
                        Debug.Assert(!manifold.contact1.id.Equals(contact.id), "Duplicate contact ids within the same manifold");
                    }
                }
            }
        }
    }

    private Lambdas GetLambdas(Geometry.ContactId id) {
        Lambdas lambdas;
        if (CollisionSystem.warmStarting && prevLambdas.TryGetValue(id, out var l)) {
            lambdas = l;
        } else {
            lambdas = new Lambdas();
        }
        return lambdas;
    }

    public void PreSteps(float dt) {
        for (int j = 0; j < boxBoxConstraints.Length; j++) {
            var c = boxBoxConstraints[j];

            helper.PreStep(ref c, dt, GetLambdas(c.id));

            // TODO: Non readonly structs are EVIL
            boxBoxConstraints[j] = c;
        }
    }

    public void ApplyImpulses(float dt) {

        for (int j = 0; j < boxBoxConstraints.Length; j++) {
            var c = boxBoxConstraints[j];

            helper.ApplyImpulse(ref c, dt);

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

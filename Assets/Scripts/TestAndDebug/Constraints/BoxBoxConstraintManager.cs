using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using UnityEngine;

using Physics.Math;

// This is what the box-box constraint manager class looks like
//      using BoxBoxConstraintManager = 
//          ConstraintManager<BoxBoxConstraintHelper, StandardConstraint>;

public struct BoxBoxConstraintHelper : ConstraintManagerHelper<StandardConstraint> {
    private ComponentDataFromEntity<Box> boxes;
    private ComponentDataFromEntity<Velocity> boxVels;
    private NativeList<Entity> boxEntities;

    public void Update(
        ComponentDataFromEntity<Box> boxes,
        ComponentDataFromEntity<Velocity> boxVels,
        NativeList<Entity> boxEntities) {

        this.boxes = boxes;
        this.boxVels = boxVels;
        this.boxEntities = boxEntities;
    }


    private Geometry.Manifold? GetManifold(Entity box1, Entity box2) {
        return Geometry.GetIntersectData(
            boxes[box1].ToRect(),
            boxes[box2].ToRect()
        );
    }

    private StandardConstraint GetConstraint(Entity box1, Entity box2, Geometry.Manifold manifold, bool useContact1) {
        return new StandardConstraint(
            box1, box2,
            manifold,
            useContact1
        );
    }

    public void ApplyImpulse(ref StandardConstraint constraint, float dt) {
        var v1 = boxVels[constraint.box1];
        var v2 = boxVels[constraint.box2];

        constraint.ApplyImpulse(ref v1, ref v2, dt);

        boxVels[constraint.box1] = v1;
        boxVels[constraint.box2] = v2;
    }

    public void PreStep(ref StandardConstraint constraint, float dt, Lambdas lambdas) {
        var v1 = boxVels[constraint.box1];
        var v2 = boxVels[constraint.box2];

        constraint.PreStep(boxes[constraint.box1], boxes[constraint.box2], ref v1, ref v2, dt, lambdas);

        boxVels[constraint.box1] = v1;
        boxVels[constraint.box2] = v2;
    }

    public void FillWithConstraints(NativeList<StandardConstraint> constraints) {
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


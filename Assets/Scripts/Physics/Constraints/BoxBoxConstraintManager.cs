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
    private NativeArray<Entity> boxEntities;
    private float dt;

    public void Update(
        ComponentDataFromEntity<Box> boxes,
        ComponentDataFromEntity<Velocity> boxVels,
        NativeArray<Entity> boxEntities,
        float dt) {

        this.boxes = boxes;
        this.boxVels = boxVels;
        this.boxEntities = boxEntities;
        this.dt = dt;
    }


    private Geometry.Manifold? GetManifold(Entity box1, Entity box2) {
        return Geometry.GetIntersectData(
            boxes[box1].ToRect(),
            boxes[box2].ToRect()
        );
    }

    private void AddConstraint(ref NativeList<StandardConstraint> constraints, Entity e1, Entity e2, Geometry.Manifold manifold, bool useContact1) {
        Box box1 = boxes[e1];
        Box box2 = boxes[e2];

        if (box1.mass == math.INFINITY && box2.mass == math.INFINITY) {
            return;
        }

        constraints.Add(new StandardConstraint(
            e1, e2,
            box1, box2,
            manifold,
            useContact1,
            dt
        ));
    }

    public void ApplyImpulse(ref StandardConstraint constraint, float dt) {
        var v1 = boxVels[constraint.e1];
        var v2 = boxVels[constraint.e2];

        constraint.ApplyImpulse(ref v1, ref v2, dt);

        boxVels[constraint.e1] = v1;
        boxVels[constraint.e2] = v2;
    }

    public void PreStep(ref StandardConstraint constraint, float dt, Lambdas lambdas) {
        var v1 = boxVels[constraint.e1];
        var v2 = boxVels[constraint.e2];

        constraint.PreStep(ref v1, ref v2, dt, lambdas);

        boxVels[constraint.e1] = v1;
        boxVels[constraint.e2] = v2;
    }

    public void FillWithConstraints(NativeList<StandardConstraint> constraints) {
        for (int i = 0; i < boxEntities.Length; i++ ) {
            for (int j = i+1; j < boxEntities.Length; j++ ) {
                Entity box1 = boxEntities[i];
                Entity box2 = boxEntities[j];

                var manifoldNullable = GetManifold(box1, box2);

                if (manifoldNullable is Geometry.Manifold manifold) {


                    AddConstraint(ref constraints, box1, box2, manifold, true);

                    if (manifold.contact2 is Geometry.Contact contact) {

                        AddConstraint(ref constraints, box1, box2, manifold, false);
                        Debug.Assert(!manifold.contact1.id.Equals(contact.id), "Duplicate contact ids within the same manifold");
                    }
                }
            }
        }
    }
}


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
    private ComponentDataFromEntity<Velocity> boxVels;

    public void Update(
            ComponentDataFromEntity<Velocity> boxVels
        ) {

        this.boxVels = boxVels;
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
}


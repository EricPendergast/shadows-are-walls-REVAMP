using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using UnityEngine;

using Physics.Math;

using ShadowEdgeManifold = ShadowEdgeGenerationSystem.ShadowEdgeManifold;

public struct ShadowEdgeConstraintHelper : ConstraintManagerHelper<ShadowEdgeConstraint> {
    private ComponentDataFromEntity<Velocity> vels;
    private ComponentDataFromEntity<Mass> masses;
    private ComponentDataFromEntity<Box> boxes;
    private ComponentDataFromEntity<LightSource> lightSources;
    private NativeArray<ShadowEdgeConstraint.Partial> partialConstraints;
    private float dt;

    public void Update(
            ComponentDataFromEntity<Velocity> vels,
            ComponentDataFromEntity<Mass> masses,
            ComponentDataFromEntity<Box> boxes,
            ComponentDataFromEntity<LightSource> lightSources,
            NativeArray<ShadowEdgeConstraint.Partial> partialConstraints,
            float dt) {
        this.vels = vels;
        this.masses = masses;
        this.boxes = boxes;
        this.lightSources = lightSources;
        this.partialConstraints = partialConstraints;
        this.dt = dt;
    }

    public void ApplyImpulse(ref ShadowEdgeConstraint constraint, float dt) {
        var v1 = vels[constraint.e1];
        var v2 = vels[constraint.e2];

        constraint.ApplyImpulse(ref v1, ref v2, dt);

        vels[constraint.e1] = v1;
        vels[constraint.e2] = v2;
    }

    public void PreStep(ref ShadowEdgeConstraint constraint, float dt, Lambdas lambdas) {
        var v1 = vels[constraint.e1];
        var v2 = vels[constraint.e2];

        constraint.PreStep(ref v1, ref v2, dt, lambdas);

        vels[constraint.e1] = v1;
        vels[constraint.e2] = v2;
    }

    public void FillWithConstraints(NativeList<ShadowEdgeConstraint> constraints) {
        foreach (var pConstraint in partialConstraints) {
            constraints.Add(new ShadowEdgeConstraint(in pConstraint, masses, dt));
        }
    }
}

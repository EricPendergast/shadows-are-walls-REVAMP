using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using UnityEngine;

using Physics.Math;

using ShadowEdge = ShadowEdgeGenerationSystem.ShadowEdge;
using ShadowEdgeManifold = ShadowEdgeGenerationSystem.ShadowEdgeManifold;
using Manifold = Physics.Math.Geometry.Manifold;

public struct ShadowEdgeConstraintHelper : ConstraintManagerHelper<ShadowEdgeConstraint> {
    private ComponentDataFromEntity<Velocity> vels;
    private ComponentDataFromEntity<Box> boxes;
    private NativeArray<Entity> hitShadBoxEntities;
    private NativeArray<ShadowEdgeManifold> shadowEdgeManifolds;
    private float dt;

    public void Update(
            ComponentDataFromEntity<Velocity> vels,
            ComponentDataFromEntity<Box> boxes,
            NativeArray<Entity> hitShadBoxEntities,
            NativeArray<ShadowEdgeManifold> shadowEdgeManifolds,
            float dt) {
        this.vels = vels;
        this.boxes = boxes;
        this.hitShadBoxEntities = hitShadBoxEntities;
        this.shadowEdgeManifolds = shadowEdgeManifolds;
        this.dt = dt;
    }

    private void AddConstraints(ref NativeList<ShadowEdgeConstraint> constraints, ShadowEdgeManifold seManifold, bool useContact1) {
        var box = seManifold.box;
        var shadowEdge = seManifold.shadowEdge;
        var manifold = seManifold.manifold;

        constraints.Add(new ShadowEdgeConstraint(
            e1: box, 
            e2: shadowEdge.opaque,
            box1: boxes[box], 
            box2: boxes[shadowEdge.opaque],
            shadowOrigin: shadowEdge.contact1,
            lightOrigin: shadowEdge.lightSource,
            manifold: manifold,
            useContact1: useContact1,
            dt: dt,
            contactIdScrambler: 1
        ));

        if (shadowEdge.contact2 is float2 shadowOrigin) {
            constraints.Add(new ShadowEdgeConstraint(
                e1: box, 
                e2: shadowEdge.opaque,
                box1: boxes[box], 
                box2: boxes[shadowEdge.opaque],
                shadowOrigin: shadowOrigin,
                lightOrigin: shadowEdge.lightSource,
                manifold: manifold,
                useContact1: useContact1,
                dt: dt,
                contactIdScrambler: 2
            ));
        }
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
        foreach (ShadowEdgeManifold seManifold in shadowEdgeManifolds) {
            Geometry.Manifold manifold = seManifold.manifold;
            AddConstraints(ref constraints, seManifold, true);

            if (manifold.contact2 is Geometry.Contact contact) {

                AddConstraints(ref constraints, seManifold, false);

                Debug.Assert(!manifold.contact1.id.Equals(contact.id), "Duplicate contact ids within the same manifold");
            }
        }
    }
}

using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using UnityEngine;

using Physics.Math;

using ShadowEdge = ShadowEdgeGenerationSystem.ShadowEdge;
using Manifold = Physics.Math.Geometry.Manifold;

public struct ShadowEdgeConstraintHelper : ConstraintManagerHelper<ShadowEdgeConstraint> {
    private ComponentDataFromEntity<Velocity> vels;
    private ComponentDataFromEntity<Box> boxes;
    private NativeList<ShadowEdge> shadowEdges;
    private NativeArray<Entity> hitShadBoxEntities;
    private float dt;

    public void Update(
            ComponentDataFromEntity<Velocity> vels,
            ComponentDataFromEntity<Box> boxes,
            NativeList<ShadowEdge> shadowEdges,
            NativeArray<Entity> hitShadBoxEntities,
            float dt) {
        this.vels = vels;
        this.boxes = boxes;
        this.shadowEdges = shadowEdges;
        this.hitShadBoxEntities = hitShadBoxEntities;
        this.dt = dt;
    }


    private Manifold? GetManifold(Entity box, ShadowEdge shadowEdge) {
        return Geometry.GetIntersectData(boxes[box].ToRect(), shadowEdge.collider);
    }

    private void AddConstraints(ref NativeList<ShadowEdgeConstraint> constraints, Entity box, ShadowEdge shadowEdge, Manifold manifold, bool useContact1) {

        constraints.Add(new ShadowEdgeConstraint(
            e1: box, 
            e2: shadowEdge.opaque,
            box1: boxes[box], 
            box2: boxes[shadowEdge.opaque],
            shadowOrigin: shadowEdge.contact1,
            lightOrigin: shadowEdge.lightSource,
            manifold: manifold,
            useContact1: useContact1,
            dt: dt
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
                dt: dt
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
        for (int i = 0; i < hitShadBoxEntities.Length; i++ ) {
            for (int j = 0; j < shadowEdges.Length; j++ ) {
                Entity box = hitShadBoxEntities[i];
                ShadowEdge shadowEdge = shadowEdges[j];

                var manifoldNullable = GetManifold(box, shadowEdge);

                if (manifoldNullable is Geometry.Manifold manifold) {

                    AddConstraints(ref constraints, box, shadowEdge, manifold, true);

                    if (manifold.contact2 is Geometry.Contact contact) {

                        AddConstraints(ref constraints, box, shadowEdge, manifold, false);

                        Debug.Assert(!manifold.contact1.id.Equals(contact.id), "Duplicate contact ids within the same manifold");
                    }
                }
            }
        }
    }
}

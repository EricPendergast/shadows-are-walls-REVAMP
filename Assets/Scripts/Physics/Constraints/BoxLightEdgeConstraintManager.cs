using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using UnityEngine;

using Physics.Math;

// This is what the box-lightEdge constraint manager class looks like
//      using BoxLightEdgeConstraintManager = 
//          ConstraintManager<BoxLightEdgeConstraintHelper, StandardConstraint>;

public struct BoxLightEdgeConstraintHelper : ConstraintManagerHelper<StandardConstraint> {
    private ComponentDataFromEntity<Box> boxes;
    private ComponentDataFromEntity<LightEdge> lightEdges;
    private ComponentDataFromEntity<Velocity> vels;
    private ComponentDataFromEntity<LightSource> lightSources;
    private NativeArray<Entity> hitShadBoxEntities;
    private NativeArray<Entity> lightEdgeEntities;
    private float dt;

    public void Update(
        ComponentDataFromEntity<Box> boxes,
        ComponentDataFromEntity<LightEdge> lightEdges,
        ComponentDataFromEntity<Velocity> vels,
        ComponentDataFromEntity<LightSource> lightSources,
        NativeArray<Entity> hitShadBoxEntities,
        NativeArray<Entity> lightEdgeEntities,
        float dt) {

        this.boxes = boxes;
        this.lightEdges = lightEdges;
        this.vels = vels;
        this.lightSources = lightSources;
        this.hitShadBoxEntities = hitShadBoxEntities;
        this.lightEdgeEntities = lightEdgeEntities;
        this.dt = dt;
    }

    private Geometry.Manifold? GetManifold(Entity boxEntity, Entity lightEdgeEntity, out Entity lightSourceOut) {
        LightEdge le = lightEdges[lightEdgeEntity];
        LightSource lightSource = lightSources[le.lightSource];

        lightSourceOut = le.lightSource;

        return Geometry.GetIntersectData(
            boxes[boxEntity].ToRect(),
            le.ToRect(ref lightSource)
        );
    }

    private StandardConstraint GetConstraint(Entity box, Entity lightSource, Geometry.Manifold manifold, bool useContact1) {
        return new StandardConstraint(
            box, lightSource,
            boxes[box], lightSources[lightSource],
            manifold,
            useContact1,
            dt
        );
    }

    public void ApplyImpulse(ref StandardConstraint constraint, float dt) {
        var v1 = vels[constraint.e1];
        var v2 = vels[constraint.e2];

        constraint.ApplyImpulse(ref v1, ref v2, dt);

        vels[constraint.e1] = v1;
        vels[constraint.e2] = v2;
    }

    public void PreStep(ref StandardConstraint constraint, float dt, Lambdas lambdas) {
        var v1 = vels[constraint.e1];
        var v2 = vels[constraint.e2];

        constraint.PreStep(ref v1, ref v2, dt, lambdas);

        vels[constraint.e1] = v1;
        vels[constraint.e2] = v2;
    }

    public void FillWithConstraints(NativeList<StandardConstraint> constraints) {
        for (int i = 0; i < hitShadBoxEntities.Length; i++ ) {
            for (int j = 0; j < lightEdgeEntities.Length; j++ ) {
                Entity box = hitShadBoxEntities[i];
                Entity lightEdge = lightEdgeEntities[j];

                // Light edges are used to calculate the manifold, but the
                // light source is used during collision resolution
                var manifoldNullable = GetManifold(box, lightEdge, out Entity lightSource);

                if (manifoldNullable is Geometry.Manifold manifold) {

                    constraints.Add(GetConstraint(box, lightSource, manifold, true));

                    if (manifold.contact2 is Geometry.Contact contact) {

                        constraints.Add(GetConstraint(box, lightSource, manifold, false));
                        Debug.Assert(!manifold.contact1.id.Equals(contact.id), "Duplicate contact ids within the same manifold");
                    }
                }
            }
        }
    }
}

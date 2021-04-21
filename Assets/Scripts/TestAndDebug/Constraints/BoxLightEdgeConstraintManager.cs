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
    private NativeList<Entity> boxEntities;
    private NativeList<Entity> lightEdgeEntities;

    public void Update(
        ComponentDataFromEntity<Box> boxes,
        ComponentDataFromEntity<LightEdge> lightEdges,
        ComponentDataFromEntity<Velocity> vels,
        ComponentDataFromEntity<LightSource> lightSources,
        NativeList<Entity> boxEntities,
        NativeList<Entity> lightEdgeEntities) {

        this.boxes = boxes;
        this.lightEdges = lightEdges;
        this.vels = vels;
        this.lightSources = lightSources;
        this.boxEntities = boxEntities;
        this.lightEdgeEntities = lightEdgeEntities;
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
            manifold,
            useContact1
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

        constraint.PreStep(boxes[constraint.e1], lightSources[constraint.e2], ref v1, ref v2, dt, lambdas);

        vels[constraint.e1] = v1;
        vels[constraint.e2] = v2;
    }

    public void FillWithConstraints(NativeList<StandardConstraint> constraints) {
        for (int i = 0; i < boxEntities.Length; i++ ) {
            for (int j = 0; j < lightEdgeEntities.Length; j++ ) {
                Entity box = boxEntities[i];
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


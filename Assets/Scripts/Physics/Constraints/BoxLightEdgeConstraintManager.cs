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
    private ComponentDataFromEntity<Velocity> vels;
    private ComponentDataFromEntity<LightSource> lightSources;
    private NativeArray<Entity> hitShadBoxEntities;
    private NativeArray<Entity> lightSourceEntities;
    private float dt;

    public void Update(
        ComponentDataFromEntity<Box> boxes,
        ComponentDataFromEntity<Velocity> vels,
        ComponentDataFromEntity<LightSource> lightSources,
        NativeArray<Entity> hitShadBoxEntities,
        NativeArray<Entity> lightSourceEntities,
        float dt) {

        this.boxes = boxes;
        this.vels = vels;
        this.lightSources = lightSources;
        this.hitShadBoxEntities = hitShadBoxEntities;
        this.lightSourceEntities = lightSourceEntities;
        this.dt = dt;
    }

    private void AddConstraints(ref NativeList<StandardConstraint> constraints, Entity boxEntity, Entity lightSourceEntity, bool lightEdgeFlag) {
        var manifoldNullable = Geometry.GetIntersectData(
            boxes[boxEntity].ToRect(),
            lightEdgeFlag ? 
                lightSources[lightSourceEntity].GetMinEdgeRect() : 
                lightSources[lightSourceEntity].GetMaxEdgeRect()
        );

        if (manifoldNullable is Geometry.Manifold manifold) {

            constraints.Add(GetConstraint(boxEntity, lightSourceEntity, manifold, true));

            if (manifold.contact2 is Geometry.Contact contact) {

                constraints.Add(GetConstraint(boxEntity, lightSourceEntity, manifold, false));
                Debug.Assert(!manifold.contact1.id.Equals(contact.id), "Duplicate contact ids within the same manifold");
            }
        }
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
            for (int j = 0; j < lightSourceEntities.Length; j++ ) {
                Entity box = hitShadBoxEntities[i];
                Entity lightSource = lightSourceEntities[j];

                AddConstraints(ref constraints, box, lightSource, true);
                AddConstraints(ref constraints, box, lightSource, false);
            }
        }
    }
}


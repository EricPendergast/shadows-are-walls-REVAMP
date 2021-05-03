using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using UnityEngine;

using Physics.Math;

// This is what the box-lightEdge constraint manager class looks like
//      using BoxLightEdgeConstraintManager = 
//          ConstraintManager<BoxLightEdgeConstraintHelper, StandardConstraint>;

using ShadowEdgeManifold = ShadowEdgeGenerationSystem.ShadowEdgeManifold;

public struct BoxLightEdgeConstraintHelper : ConstraintManagerHelper<StandardConstraint> {
    private ComponentDataFromEntity<Box> boxes;
    private ComponentDataFromEntity<Velocity> vels;
    private ComponentDataFromEntity<LightSource> lightSources;
    private NativeArray<Entity> hitShadBoxEntities;
    private NativeArray<Entity> lightSourceEntities;
    private NativeArray<ShadowEdgeManifold> lightEdgeManifolds;
    private float dt;

    public void Update(
        ComponentDataFromEntity<Box> boxes,
        ComponentDataFromEntity<Velocity> vels,
        ComponentDataFromEntity<LightSource> lightSources,
        NativeArray<Entity> hitShadBoxEntities,
        NativeArray<Entity> lightSourceEntities,
        NativeArray<ShadowEdgeManifold> lightEdgeManifolds,
        float dt) {

        this.boxes = boxes;
        this.vels = vels;
        this.lightSources = lightSources;
        this.hitShadBoxEntities = hitShadBoxEntities;
        this.lightSourceEntities = lightSourceEntities;
        this.lightEdgeManifolds = lightEdgeManifolds;
        this.dt = dt;
    }

    private void AddConstraints(ref NativeList<StandardConstraint> constraints, ShadowEdgeManifold m, bool useContact1) {
        Debug.Assert(m.castingShapeType == ShapeType.Light);

        var standardManifold = new Physics.Math.Geometry.Manifold {
            contact1 = m.contact1,
            contact2 = m.contact2,
            normal = m.normal,
            overlap = m.overlap
        };

        constraints.Add(GetConstraint(m.shadHitEntity, m.castingEntity, standardManifold, true));

        if (standardManifold.contact2 is Geometry.Contact contact) {
            constraints.Add(GetConstraint(m.shadHitEntity, m.castingEntity, standardManifold, false));
            Debug.Assert(!standardManifold.contact1.id.Equals(contact.id), "Duplicate contact ids within the same manifold");
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
        foreach (ShadowEdgeManifold seManifold in lightEdgeManifolds) {
            AddConstraints(ref constraints, seManifold, true);
        }
    }
}


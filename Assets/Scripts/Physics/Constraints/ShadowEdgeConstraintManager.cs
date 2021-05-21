using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using UnityEngine;

using Physics.Math;

using ShadowEdgeManifold = ShadowEdgeGenerationSystem.ShadowEdgeManifold;

public struct ShadowEdgeConstraintHelper : ConstraintManagerHelper<ShadowEdgeConstraint> {
    private ComponentDataFromEntity<Velocity> vels;
    private ComponentDataFromEntity<Box> boxes;
    private ComponentDataFromEntity<LightSource> lightSources;
    private NativeArray<ShadowEdgeManifold> shadowEdgeManifolds;
    private float dt;

    public void Update(
            ComponentDataFromEntity<Velocity> vels,
            ComponentDataFromEntity<Box> boxes,
            ComponentDataFromEntity<LightSource> lightSources,
            NativeArray<ShadowEdgeManifold> shadowEdgeManifolds,
            float dt) {
        this.vels = vels;
        this.boxes = boxes;
        this.lightSources = lightSources;
        this.shadowEdgeManifolds = shadowEdgeManifolds;
        this.dt = dt;
    }

    private void AddConstraints(ref NativeList<ShadowEdgeConstraint> constraints, ShadowEdgeManifold m, bool useContact1) {
        var standardManifold = new Physics.Math.Geometry.Manifold{
            contact1 = m.contact1,
            contact2 = m.contact2,
            normal = m.normal,
            overlap = m.overlap
        };

        if (m.castingShapeType == ShapeType.Box) {
            constraints.Add(new ShadowEdgeConstraint(
                e1: m.shadHitEntity, 
                e2: m.castingEntity,
                box1: boxes[m.shadHitEntity], 
                box2: boxes[m.castingEntity],
                shadowOrigin: m.mount1,
                lightOrigin: m.lightSource,
                manifold: standardManifold,
                useContact1: useContact1,
                dt: dt,
                contactIdScrambler: 1
            ));

            if (m.mount2 is float2 shadowOrigin) {
                constraints.Add(new ShadowEdgeConstraint(
                    e1: m.shadHitEntity, 
                    e2: m.castingEntity,
                    box1: boxes[m.shadHitEntity], 
                    box2: boxes[m.castingEntity],
                    shadowOrigin: shadowOrigin,
                    lightOrigin: m.lightSource,
                    manifold: standardManifold,
                    useContact1: useContact1,
                    dt: dt,
                    contactIdScrambler: 2
                ));
            }
        } else if (m.castingShapeType == ShapeType.Light) {
            constraints.Add(GetConstraint(m.shadHitEntity, m.castingEntity, standardManifold, useContact1));
        } else {
            Debug.Assert(false, "Unanticipated shape type: " + m.castingShapeType);
        }
    }

    private ShadowEdgeConstraint GetConstraint(Entity box, Entity lightSource, Geometry.Manifold manifold, bool useContact1) {
        return new ShadowEdgeConstraint(
            box, lightSource,
            boxes[box], lightSources[lightSource],
            manifold,
            useContact1,
            dt
        );
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
            AddConstraints(ref constraints, seManifold, true);

            if (seManifold.contact2 is Geometry.Contact contact) {

                AddConstraints(ref constraints, seManifold, false);

                Debug.Assert(!seManifold.contact1.id.Equals(contact.id), "Duplicate contact ids within the same manifold");
            }
        }
    }
}

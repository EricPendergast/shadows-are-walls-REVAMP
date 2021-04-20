using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using UnityEngine;

using Physics.Math;

using ContactId = Physics.Math.Geometry.ContactId;

public struct BoxLightEdgeConstraintHelper : ConstraintManagerHelper<StandardConstraint> {
    private ComponentDataFromEntity<Box> boxes;
    private ComponentDataFromEntity<LightEdge> lightEdges;
    private ComponentDataFromEntity<Velocity> vels;
    private NativeList<Entity> boxEntities;
    private NativeList<Entity> lightEdgeEntities;

    public void Update(
        ComponentDataFromEntity<Box> boxes,
        ComponentDataFromEntity<LightEdge> lightEdges,
        ComponentDataFromEntity<Velocity> vels,
        NativeList<Entity> boxEntities,
        NativeList<Entity> lightEdgeEntities) {

        this.boxes = boxes;
        this.lightEdges = lightEdges;
        this.vels = vels;
        this.boxEntities = boxEntities;
        this.lightEdgeEntities = lightEdgeEntities;
    }

    private Geometry.Manifold? GetManifold(Entity box, Entity lightEdge) {
        return Geometry.GetIntersectData(
            boxes[box].ToRect(),
            lightEdges[lightEdge].ToRect()
        );
    }

    private StandardConstraint GetConstraint(Entity box, Entity lightEdge, Geometry.Manifold manifold, bool useContact1) {
        return new StandardConstraint(
            box, lightEdge,
            manifold,
            useContact1
        );
    }

    public void ApplyImpulse(ref StandardConstraint constraint, float dt) {
        var v1 = vels[constraint.box1];
        var v2 = vels[constraint.box2];

        constraint.ApplyImpulse(ref v1, ref v2, dt);

        vels[constraint.box1] = v1;
        vels[constraint.box2] = v2;
    }

    public void PreStep(ref StandardConstraint constraint, float dt, Lambdas lambdas) {
        var v1 = vels[constraint.box1];
        var v2 = vels[constraint.box2];

        constraint.PreStep(boxes[constraint.box1], lightEdges[constraint.box2], ref v1, ref v2, dt, lambdas);

        vels[constraint.box1] = v1;
        vels[constraint.box2] = v2;
    }

    public void FillWithConstraints(NativeList<StandardConstraint> constraints) {
        for (int i = 0; i < boxEntities.Length; i++ ) {
            for (int j = 0; j < lightEdgeEntities.Length; j++ ) {
                Entity box = boxEntities[i];
                Entity lightEdge = lightEdgeEntities[j];

                var manifoldNullable = GetManifold(box, lightEdge);

                if (manifoldNullable is Geometry.Manifold manifold) {

                    constraints.Add(GetConstraint(box, lightEdge, manifold, true));

                    if (manifold.contact2 is Geometry.Contact contact) {

                        constraints.Add(GetConstraint(box, lightEdge, manifold, false));
                        Debug.Assert(!manifold.contact1.id.Equals(contact.id), "Duplicate contact ids within the same manifold");
                    }
                }
            }
        }
    }
}


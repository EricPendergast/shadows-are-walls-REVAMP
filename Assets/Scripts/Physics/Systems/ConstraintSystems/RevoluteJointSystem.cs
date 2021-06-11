using System.Collections.Generic;

using Unity.Entities;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

using UnityEngine;

using Physics.Math;

[UpdateInGroup(typeof(ConstraintGenerationSystemGroup))]
public class RevoluteJointSystem : SystemBase {

    protected override void OnUpdate() {
        float dt = Time.DeltaTime;
        var boxes = GetComponentDataFromEntity<Box>();
        // TODO: I only need position, but position is baked into several
        // types. So its messy.
        var mouses = GetComponentDataFromEntity<MouseComponent>();
        var masses = GetComponentDataFromEntity<Mass>();


        var constraints = World.GetOrCreateSystem<CollisionSystem>().GetTwoWayTwoDOFConstraintsInput();

        Entities.ForEach((Entity jointEntity, in RevoluteJoint joint) => {
            constraints.Add(new TwoWayTwoDOFConstraint(
                GetManifold(jointEntity, joint, mouses, boxes),
                masses,
                dt
            ));
        }).Run();

    }

    public List<RevoluteJointManifold> GetManifoldsForDebug() {
        var ret = new List<RevoluteJointManifold>();

        var boxes = GetComponentDataFromEntity<Box>();
        var mouses = GetComponentDataFromEntity<MouseComponent>();

        Entities
            .WithoutBurst()
            .ForEach((Entity jointEntity, in RevoluteJoint joint) => {
            ret.Add(GetManifold(jointEntity, joint, mouses, boxes));
        }).Run();

        return ret;
    }

    private static RevoluteJointManifold GetManifold(Entity jointEntity, RevoluteJoint joint, ComponentDataFromEntity<MouseComponent> mouses, ComponentDataFromEntity<Box> boxes) {
        float2 GetPos(Entity e) {
            if (mouses.HasComponent(e)) {
                return mouses[e].pos;
            } else {
                return boxes[e].pos;
            }
        }

        float2 LocalToWorld(Entity e, float2 v) {
            if (mouses.HasComponent(e)) {
                return v;
            } else {
                return boxes[e].LocalVecToWorld(v);
            }
        }

        float2 x1 = GetPos(joint.e1);
        float2 x2 = GetPos(joint.e2);
        float2 r1 = LocalToWorld(joint.e1, joint.r1);
        float2 r2 = LocalToWorld(joint.e2, joint.r2);

        return new RevoluteJointManifold{
            e1 = joint.e1,
            e2 = joint.e2,
            r1 = r1,
            r2 = r2,
            delta = x2 + r2 - x1 - r1,
            id = jointEntity.GetHashCode() ^ 19378933,
            softness = joint.softness,
            beta = joint.beta
        };
    }
}
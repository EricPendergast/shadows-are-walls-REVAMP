using System.Collections.Generic;

using Unity.Entities;
using Unity.Mathematics;

[UpdateInGroup(typeof(ConstraintGenerationSystemGroup))]
public class RevoluteJointSystem : SystemBase {

    protected override void OnUpdate() {
        float dt = Time.DeltaTime;

        var masses = GetComponentDataFromEntity<Mass>();
        var positions = GetComponentDataFromEntity<Position>();

        var constraints = World.GetOrCreateSystem<CollisionSystem>().GetTwoWayTwoDOFConstraintsInput();

        Entities.ForEach((Entity jointEntity, in RevoluteJoint joint) => {
            constraints.Add(new TwoWayTwoDOFConstraint(
                GetManifold(jointEntity, joint, positions),
                masses,
                dt
            ));
        }).Run();

    }

    public List<RevoluteJointManifold> GetManifoldsForDebug() {
        var ret = new List<RevoluteJointManifold>();

        var positions = GetComponentDataFromEntity<Position>();

        Entities
            .WithoutBurst()
            .ForEach((Entity jointEntity, in RevoluteJoint joint) => {
            ret.Add(GetManifold(jointEntity, joint, positions));
        }).Run();

        return ret;
    }

    private static RevoluteJointManifold GetManifold(Entity jointEntity, RevoluteJoint joint, ComponentDataFromEntity<Position> positions) {
        var pos1 = positions[joint.e1];
        var pos2 = positions[joint.e2];

        float2 x1 = pos1.pos;
        float2 x2 = pos2.pos;
        float2 r1 = pos1.LocalDirectionToGlobal(joint.r1);
        float2 r2 = pos2.LocalDirectionToGlobal(joint.r2);

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

using Unity.Entities;
using Unity.Mathematics;

[UpdateInGroup(typeof(ConstraintGenerationSystemGroup))]
public class LightMountSystem : SystemBase {

    protected override void OnUpdate() {
        var output = World.GetOrCreateSystem<ConstraintGatherSystem>().GetTwoWayPenConstraintsInput();

        var masses = GetComponentDataFromEntity<Mass>();
        var positions = GetComponentDataFromEntity<Position>();

        float dt = Time.DeltaTime;

        Entities
        .WithReadOnly(masses)
        .WithReadOnly(positions)
        .ForEach((Entity lightEntity, in LightSource lightSource, in LightMountJoint joint, in Position lightPos) => {

            float delta = joint
                .GetAngleRange(positions[joint.mount].rot)
                .GetMinResolveOverlap(
                    lightSource.GetAngleRange(lightPos.rot)
                );

            if (delta != 0) {
                var m = new AnglePenetrationManifold{
                    e1 = lightEntity,
                    e2 = joint.mount,
                    id = new int2(lightEntity.GetHashCode(), joint.mount.GetHashCode()).GetHashCode() ^ 280232179,
                    softness = 0,
                    beta = .2f,
                    delta = delta
                };
                output.Add(new TwoWayPenConstraint(m, masses, dt));
            }
        }).Schedule();
    }
}

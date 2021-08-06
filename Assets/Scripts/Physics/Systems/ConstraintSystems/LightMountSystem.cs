using Unity.Entities;
using Unity.Mathematics;

[UpdateInGroup(typeof(ContactGenerationGroup))]
public class LightMountSystem : SystemBase {

    protected override void OnUpdate() {
        var output = World.GetOrCreateSystem<ConstraintGatherSystem>().GetTwoWayPenConstraintsInput();

        var masses = GetComponentDataFromEntity<Mass>();
        var positions = GetComponentDataFromEntity<Position>();
        var lightSources = GetComponentDataFromEntity<LightSource>();

        float dt = Time.DeltaTime;

        Entities
        .WithReadOnly(masses)
        .WithReadOnly(positions)
        .ForEach((in LightMountJoint joint) => {
            var lightSource = lightSources[joint.lightEntity];
            var lightPos = positions[joint.lightEntity];

            float delta = joint
                .GetAngleRange(positions[joint.mount].rot)
                .GetMinResolveOverlap(
                    lightSource.GetAngleRange(lightPos.rot)
                );

            if (delta != 0) {
                var m = new AnglePenetrationManifold{
                    e1 = joint.lightEntity,
                    e2 = joint.mount,
                    id = joint.id,
                    softness = 0,
                    beta = .2f,
                    delta = delta
                };
                output.Add(new TwoWayPenConstraint(m, masses, dt));
            }
        }).Schedule();
    }
}

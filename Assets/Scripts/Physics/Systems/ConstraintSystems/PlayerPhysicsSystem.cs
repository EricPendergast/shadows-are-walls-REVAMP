using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;


[UpdateInGroup(typeof(ConstraintGenerationSystemGroup), OrderFirst=true)]
public class PlayerPhysicsSystem : SystemBase {

    protected override void OnCreate() {
        RequireSingletonForUpdate<PlayerSettings>();
    }

    protected override void OnUpdate() {

        var input = World.GetOrCreateSystem<ConstraintGatherSystem>()
            .GetOneWayOneDOFConstraintsInput();
        var masses = GetComponentDataFromEntity<Mass>();
        var dt = Time.DeltaTime;

        var settings = GetSingleton<PlayerSettings>();

        Entities.ForEach((Entity e, in PlayerComponent pc, in Box playerBox, in Position playerPos) => {
            if (pc.moveDirection == 0) {
                return;
            }

            var manifold = new TargetVelocityManifold {
                softness = settings.softness,
                id = e.GetHashCode(),
                r = float2.zero,
                e = e,
                normal = new float2(pc.moveDirection, 0),
                targetSpeed = settings.moveSpeed
            };
            input.Add(new OneWayOneDOFConstraint(manifold, masses, dt));
        }).Run();
    }
}

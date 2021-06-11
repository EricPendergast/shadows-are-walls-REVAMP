using Unity.Entities;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(CollisionSystem))]
public class VelocityIntegrationSystem : SystemBase {
    protected override void OnUpdate() {
        float dt = Time.DeltaTime;
        Entities.ForEach((ref Position pos, in Velocity vel) => {
            pos.pos += dt*vel.vel;
            pos.rot += dt*vel.angVel;
        }).Run();
    }
}

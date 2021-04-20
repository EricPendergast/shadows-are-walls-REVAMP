using Unity.Entities;
using Unity.Transforms;
using Unity.Collections;
using Unity.Mathematics;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(CollisionSystem))]
public class VelocityIntegrationSystem : SystemBase {
    protected override void OnUpdate() {
        float dt = Time.DeltaTime;
        Entities.ForEach((ref Box box, in Velocity vel) => {
            box.pos += dt*vel.vel;
            box.rot += dt*vel.angVel;
        }).Run();
        Entities.ForEach((ref LightEdge le, in Velocity vel) => {
            le.pos += dt*vel.vel;
            le.rot += dt*vel.angVel;
        }).Run();
    }
}

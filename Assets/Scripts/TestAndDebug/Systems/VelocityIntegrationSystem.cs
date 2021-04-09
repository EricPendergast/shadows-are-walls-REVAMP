using Unity.Entities;
using Unity.Transforms;
using Unity.Collections;
using Unity.Mathematics;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(CollisionSystem))]
public class VelocityIntegrationSystem : SystemBase {
    protected override void OnUpdate() {
        float dt = Time.DeltaTime;
        Entities.ForEach((ref Box box) => {
            box.pos += dt*box.vel;
            box.rot += dt*box.angVel;
        }).Run();
    }
}

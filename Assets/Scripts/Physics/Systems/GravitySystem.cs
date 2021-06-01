using Unity.Entities;
using Unity.Transforms;
using Unity.Collections;
using Unity.Mathematics;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(PrePhysicsSystemGroup))]
public class GravitySystem : SystemBase {
    protected override void OnUpdate() {
        float2 gravityAccel = new float2(0, -10f);
        float dt = Time.DeltaTime;
        Entities.
            WithNone<GravityScale>().
            ForEach((ref Velocity vel) => {
               vel.vel += dt*gravityAccel;
        }).Run();

        Entities.
            ForEach((ref Velocity vel, in GravityScale gs) => {
                vel.vel += dt*gravityAccel*gs.gravityScale;
        }).Run();
    }
}

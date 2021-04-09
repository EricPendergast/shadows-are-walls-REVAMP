using Unity.Entities;
using Unity.Transforms;
using Unity.Collections;
using Unity.Mathematics;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
public class GravitySystem : SystemBase {
    protected override void OnUpdate() {
        float2 gravityAccel = new float2(0, -9.8f);
        float dt = Time.DeltaTime;
        Entities.
            WithNone<GravityScale>().
            ForEach((ref Box box) => {
                box.vel += dt*gravityAccel;
        }).Run();

        Entities.
            ForEach((ref Box box, in GravityScale gs) => {
                box.vel += dt*gravityAccel*gs.gravityScale;
        }).Run();
    }
}

using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;

// Syncs the physics state with the rendering state.
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(CollisionSystem))]
[UpdateBefore(typeof(VelocityIntegrationSystem))]
public class BoxRenderSystem : SystemBase {
    protected override void OnUpdate() {
        Entities
            .ForEach((ref Translation pos, ref Rotation rot, in Box box, in Position boxPos) => {
                pos.Value = new float3(boxPos.pos, 0);
                rot.Value = quaternion.Euler(0, 0, boxPos.rot);
            }).Run();
    }
}

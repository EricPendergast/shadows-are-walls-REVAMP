using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;

// Syncs the physics state with the rendering state.
public class BoxRenderSystem : SystemBase {
    protected override void OnUpdate() {
        Entities
            .ForEach((ref Translation pos, ref Rotation rot, in Box box) => {
                
                pos.Value = new float3(box.pos, 0);
                rot.Value = quaternion.Euler(0, 0, box.rot);
            }).ScheduleParallel();
    }
}

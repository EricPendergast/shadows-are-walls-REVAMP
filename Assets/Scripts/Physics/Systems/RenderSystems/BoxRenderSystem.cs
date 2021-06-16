using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;

// Syncs the physics state with the rendering state.
[UpdateInGroup(typeof(RenderSystemGroup))]
public class BoxRenderSystem : SystemBase {
    protected override void OnUpdate() {
        Entities
            .ForEach((ref LocalToWorld l2w, in Box box, in Position boxPos) => {
                l2w.Value = float4x4.TRS(
                    new float3(boxPos.pos, 0),
                    quaternion.RotateZ(boxPos.rot),
                    new float3(math.length(box.width), math.length(box.height), 1)
                );
            }).Run();
    }
}

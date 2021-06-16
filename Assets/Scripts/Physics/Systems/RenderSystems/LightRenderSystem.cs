using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;

// Syncs the physics state with the rendering state.
[UpdateInGroup(typeof(RenderSystemGroup))]
public class LightRenderSystem : SystemBase {
    protected override void OnUpdate() {
        ShadowRenderPassFeature.lights.Clear();

        Entities
            .WithoutBurst()
            .ForEach((ref LocalToWorld rot, in LightSource lightSource, in Position lightPos) => {
                rot.Value = float4x4.TRS(
                    new float3(lightPos.pos, 0),
                    quaternion.RotateZ(lightPos.rot),
                    new float3(1, 1, 1)
                );

                ShadowRenderPassFeature.lights.Add(lightSource.GetLightMatrix(lightPos));
            }).Run();
    }
}


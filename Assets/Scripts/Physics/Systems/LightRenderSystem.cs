using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using Unity.Rendering;
using UnityEngine;
using Physics.Math;

using Utilities;
// Syncs the physics state with the rendering state.
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(ShadowEdgeGenerationSystem))]
[UpdateBefore(typeof(VelocityIntegrationSystem))]
public class LightRenderSystem : SystemBase {
    protected override void OnUpdate() {
        List<Matrix4x4> lights = new List<Matrix4x4>(50);

        Entities
            .WithoutBurst()
            .ForEach((ref Translation pos, ref Rotation rot, in LightSource lightSource) => {
                pos.Value = new float3(lightSource.pos, 0);
                rot.Value = quaternion.Euler(0, 0, lightSource.rot);
                Matrix4x4 light = new Matrix4x4(
                    (Vector2)lightSource.pos,
                    (Vector2)lightSource.GetLeadingEdgeNorm(),
                    (Vector2)lightSource.GetTrailingEdgeNorm(),
                    Vector4.zero
                );
                lights.Add(light);
            }).Run();

        int lightsCount = lights.Count;
        // Since you can't resize a matrix array, we need to allocate the max amount right away.
        for (int i = lightsCount; i < 50; i++) {
            lights.Add(Matrix4x4.zero);
        }

        Shader.SetGlobalMatrixArray(GlobalShaderProperties.lights, lights);
        Shader.SetGlobalInt(GlobalShaderProperties.numLights, lightsCount);
    }
}

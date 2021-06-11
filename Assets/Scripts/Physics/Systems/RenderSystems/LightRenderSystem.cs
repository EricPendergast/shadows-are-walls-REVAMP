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
[UpdateAfter(typeof(CollisionSystem))]
[UpdateBefore(typeof(VelocityIntegrationSystem))]
public class LightRenderSystem : SystemBase {
    protected override void OnUpdate() {
        ShadowRenderPassFeature.lights.Clear();

        Entities
            .WithoutBurst()
            .ForEach((ref Translation t, ref Rotation rot, in LightSource lightSource, in Position lightPos) => {
                t.Value = new float3(lightPos.pos, 0);
                rot.Value = quaternion.Euler(0, 0, lightPos.rot);
                ShadowRenderPassFeature.lights.Add(lightSource.GetLightMatrix(lightPos));
            }).Run();
    }
}


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
        Entities
            .ForEach((ref Translation pos, ref Rotation rot, in LightSource lightSource) => {
                pos.Value = new float3(lightSource.pos, 0);
                rot.Value = quaternion.Euler(0, 0, lightSource.rot);
            }).Run();

        var lightSources = GetComponentDataFromEntity<LightSource>(false);

        var shadowEdgeSystem = World.GetOrCreateSystem<ShadowEdgeGenerationSystem>();

        //Entities
        //    .WithStructuralChanges()
        //    .ForEach((ref RenderBounds bounds, in LightSource light, in Entity entity) => {
        //
        //        RenderMesh renderMesh = EntityManager.GetSharedComponentData<RenderMesh>(entity);
        //
        //        renderMesh.mesh.Clear();
        //
        //        List<Vector3> verts = new List<Vector3>();
        //        List<int> triangles = new List<int>();
        //
        //        foreach (var point in shadowEdgeSystem.GetRenderPoints(entity)) {
        //            verts.Add((Vector2)light.GlobalToLocal(point));
        //        }
        //        
        //        for (int i = 0; i < verts.Count; i++) {
        //            triangles.Add(i);
        //        }
        //
        //        renderMesh.mesh.vertices = verts.ToArray();
        //        renderMesh.mesh.triangles = triangles.ToArray();
        //
        //        renderMesh.mesh.RecalculateBounds();
        //        bounds.Value = renderMesh.mesh.bounds.ToAABB();
        //    }
        //).Run();
    }
}

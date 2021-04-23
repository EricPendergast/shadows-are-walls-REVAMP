using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using Unity.Rendering;
using UnityEngine;

using Utilities;
// Syncs the physics state with the rendering state.
[UpdateInGroup(typeof(SimulationSystemGroup))]
public class LightRenderSystem : SystemBase {
    protected override void OnUpdate() {
        Entities
            .ForEach((ref Translation pos, ref Rotation rot, in LightSource lightSource) => {
                pos.Value = new float3(lightSource.pos, 0);
                rot.Value = quaternion.Euler(0, 0, lightSource.rot);
            }).Run();

        var lightSources = GetComponentDataFromEntity<LightSource>(false);

        var vertices = new NativeMultiHashMap<Entity, float2>(0, Allocator.TempJob);

        Entities
            .ForEach((in LightEdge lightEdge) => {
                LightSource source = lightSources[lightEdge.lightSource];
                var rect = lightEdge.ToRect(ref source);

                vertices.Add(lightEdge.lightSource, math.mul(quaternion.Euler(0, 0, lightEdge.rot), new float3(20, 0, 0)).xy);
            }).Run();


        Entities
            .WithStructuralChanges()
            .ForEach((ref RenderBounds bounds, in LightSource light, in Entity entity) => {

                RenderMesh renderMesh = EntityManager.GetSharedComponentData<RenderMesh>(entity);

                renderMesh.mesh.Clear();

                List<Vector3> verts = new List<Vector3>();
                List<int> triangles = new List<int>();

                verts.Add(Vector2.zero);
                foreach (float2 vertex in It.Iterate(vertices, entity)) {
                    verts.Add((Vector2)vertex);
                }
                
                for (int i = 1; i+1 < verts.Count; i++ ) {
                    triangles.Add(0);
                    triangles.Add(i);
                    triangles.Add(i+1);
                }

                renderMesh.mesh.vertices = verts.ToArray();
                renderMesh.mesh.triangles = triangles.ToArray();

                renderMesh.mesh.RecalculateBounds();
                bounds.Value = renderMesh.mesh.bounds.ToAABB();

            }).Run();

        vertices.Dispose();
    }
}

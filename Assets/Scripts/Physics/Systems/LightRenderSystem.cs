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

        // TODO: The way I'm doing this doesn't make much sense, because of
        // refactoring elsewhere and not wanting to change too much here. But
        // it may make more sense when I make future changes.

        Entities
            .ForEach((in LightSource lightSource, in Entity entity) => {
                vertices.Add(entity, Lin.Rotate(lightSource.GetMaxEdgeNorm()*200, -lightSource.rot));
                vertices.Add(entity, Lin.Rotate(lightSource.GetMinEdgeNorm()*200, -lightSource.rot));
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
                    verts.Add(new Vector3(vertex.x, vertex.y, 1));
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

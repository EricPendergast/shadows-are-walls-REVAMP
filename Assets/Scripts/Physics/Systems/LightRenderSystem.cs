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

        Entities
            .WithStructuralChanges()
            .ForEach((ref RenderBounds bounds, in LightSource light, in Entity entity) => {

                RenderMesh renderMesh = EntityManager.GetSharedComponentData<RenderMesh>(entity);

                renderMesh.mesh.Clear();

                List<Vector3> verts = new List<Vector3>();
                List<int> triangles = new List<int>();

                verts.Add(Vector2.zero);
                // TODO: This is going to be the end of the min light edge
                verts.Add(Vector2.zero);
                foreach (var edge in shadowEdgeSystem.GetShadowEdges(entity)) {
                    float2 p1;
                    float2 p2;
                    if (edge.leading) {
                        p1 = edge.CalculateEndPoint();
                        p2 = edge.contact1;
                    } else {
                        p1 = edge.contact1;
                        p2 = edge.CalculateEndPoint();
                    }

                    verts.Add((Vector2)light.GlobalToLocal(p1));
                    verts.Add((Vector2)light.GlobalToLocal(p2));
                }
                // TODO: This is going to be the end of the max light edge
                verts.Add(Vector2.zero);
                
                for (int i = 1; i+1 < verts.Count; i+=2) {
                    triangles.Add(0);
                    triangles.Add(i);
                    triangles.Add(i+1);
                }

                renderMesh.mesh.vertices = verts.ToArray();
                renderMesh.mesh.triangles = triangles.ToArray();

                renderMesh.mesh.RecalculateBounds();
                bounds.Value = renderMesh.mesh.bounds.ToAABB();

            }
        ).Run();
    }
}

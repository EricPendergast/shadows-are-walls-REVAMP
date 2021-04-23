using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using Physics.Math;
using UnityEngine;

using Rect = Physics.Math.Rect;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(GravitySystem))]
public class ShadowEdgeGenerationSystem : SystemBase {
    private EntityQuery lightSourceQuery;

    public struct ShadowEdge {
        public float2 contact1;
        public float2? contact2;
        public Entity opaque;
        public Rect collider;
        public float2 lightSource;
    }
    NativeList<ShadowEdge> shadowEdges;

    protected override void OnCreate() {
        shadowEdges = new NativeList<ShadowEdge>(Allocator.Persistent);
        lightSourceQuery = GetEntityQuery(typeof(LightSource));
    }

    protected override void OnDestroy() {
        shadowEdges.Dispose();
    }
    
    protected override void OnUpdate() {
        var shadowEdges = this.shadowEdges;
        shadowEdges.Clear();
        var lightSources = lightSourceQuery.ToComponentDataArray<LightSource>(Allocator.TempJob);
        // TODO: Proper contact id generation

        foreach (var l in lightSources) {
            float2 lightPos = l.pos;
            Entities
                .WithAll<OpaqueObject>()
                .ForEach((in Box box, in Entity entity) => {
                    Geometry.CalculateShadowGeometry(box.ToRect(), lightPos, .05f, out var sg1, out var sg2);
                    Debug.Log("Added shadow edge");
                    shadowEdges.Add(new ShadowEdge {
                        contact1 = sg1.contact1,
                        contact2 = sg1.contact2,
                        opaque = entity,
                        collider = Rect.FromLineSegment(sg1.contact1, sg1.contact1 + math.normalize(sg1.contact1 - lightPos)*20, 123456),
                        lightSource = lightPos
                    });
                    shadowEdges.Add(new ShadowEdge {
                        contact1 = sg2.contact1,
                        contact2 = sg2.contact2,
                        opaque = entity,
                        collider = Rect.FromLineSegment(sg2.contact1, sg2.contact1 + math.normalize(sg2.contact1 - lightPos)*20, 123456),
                        lightSource = lightPos
                    });
                }
            ).Run();
        }

        lightSources.Dispose();
    }

    public IEnumerable<ShadowEdge> GetShadowEdgesForDebug() {
        foreach (var edge in shadowEdges) {
            yield return edge;
        }
    }

    public NativeList<ShadowEdge> GetShadowEdges() {
        return shadowEdges;
    }
}

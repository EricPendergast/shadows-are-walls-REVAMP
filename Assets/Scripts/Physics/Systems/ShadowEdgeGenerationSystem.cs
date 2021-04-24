using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using Physics.Math;
using UnityEngine;

using Rect = Physics.Math.Rect;

using ShadowEdge = ShadowEdgeGenerationSystem.ShadowEdge;

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

    List<LightManager> lightManagers;

    protected override void OnCreate() {
        lightSourceQuery = GetEntityQuery(typeof(LightSource));
        lightManagers = new List<LightManager>();
    }

    protected override void OnDestroy() {
        Clear(lightManagers);
    }

    protected override void OnUpdate() {
        Clear(lightManagers);

        var lightSources = lightSourceQuery.ToComponentDataArray<LightSource>(Allocator.TempJob);

        foreach (var lightSource in lightSources) {
            lightManagers.Add(new LightManager(in lightSource));
        }
        
        Entities
            .WithAll<OpaqueObject>()
            .WithoutBurst()
            .ForEach((in Box box, in Entity entity) => {
                foreach (var l in lightManagers) {
                    l.Add(in box, in entity);
                }
            }).Run();
        
        foreach (var lightManager in lightManagers) {
            lightManager.Compute();
        }
        
        lightSources.Dispose();
    }

    public IEnumerable<ShadowEdge> GetShadowEdgesForDebug() {
        foreach (var edge in lightManagers[0].shadowEdges) {
            yield return edge;
        }
    }

    public NativeList<ShadowEdge> GetShadowEdges() {
        // TODO: For now, there is only one light manager
        return lightManagers[0].shadowEdges;
    }

    private void Clear(List<LightManager> lightManagers) {
        foreach (var lightManager in lightManagers) {
            lightManager.Dispose();
        }
        lightManagers.Clear();
    }

}

public class LightManager {
    private LightSource source;
    public NativeList<ShadowEdge> shadowEdges {get;}

    private struct ShadowEdgeData {
        float2 contact1;
        float2? contact2;
        int id;
    }
    private struct ProtoEdge {
        float angle;
        Entity opaque;
    }

    public LightManager(in LightSource source) {
        this.source = source;
        shadowEdges = new NativeList<ShadowEdge>(Allocator.TempJob);
    }

    public void Dispose() {
        shadowEdges.Dispose();
    }

    public void Add(in Box box, in Entity entity) {
        // TODO: For the purposes of calculation, we could use a smaller struct.
        float2 lightPos = source.pos;
        Geometry.CalculateShadowGeometry(box.ToRect(), lightPos, .05f, out var sg1, out var sg2);
        shadowEdges.Add(new ShadowEdge {
            contact1 = sg1.contact1,
            contact2 = sg1.contact2,
            opaque = entity,
            collider = Rect.FromLineSegment(sg1.contact1, sg1.contact1 + math.normalize(sg1.contact1 - lightPos)*20, sg1.id),
            lightSource = lightPos
        });
        shadowEdges.Add(new ShadowEdge {
            contact1 = sg2.contact1,
            contact2 = sg2.contact2,
            opaque = entity,
            collider = Rect.FromLineSegment(sg2.contact1, sg2.contact1 + math.normalize(sg2.contact1 - lightPos)*20, sg2.id),
            lightSource = lightPos
        });
    }

    public void Compute() {
        
    }
}

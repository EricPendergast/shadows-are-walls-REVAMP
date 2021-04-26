using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;
using System.Collections;

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
        // TODO: It may make sense to join these into the same term
        public float length;
        public bool leading;
        // TODO: It may be worth it to remove this field, since it is
        // technically redundant. Would have to write a special collision
        // function if we do this.
        public Rect collider;
        public float2 lightSource;
        public float2 CalculateEndPoint() {
            return contact1 + math.normalize(contact1 - lightSource)*length;
        }
    }

    Dictionary<Entity, LightManager> lightManagers;

    protected override void OnCreate() {
        lightSourceQuery = GetEntityQuery(typeof(LightSource));
        lightManagers = new Dictionary<Entity, LightManager>();
    }

    protected override void OnDestroy() {
        Clear(lightManagers);
    }

    protected override void OnUpdate() {
        Clear(lightManagers);

        var lightSources = lightSourceQuery.ToComponentDataArray<LightSource>(Allocator.TempJob);
        var lightSourceEntities = lightSourceQuery.ToEntityArray(Allocator.TempJob);
        var boxes = GetComponentDataFromEntity<Box>(true);

        for (int i = 0; i < lightSources.Length; i++) {
            lightManagers[lightSourceEntities[i]] = new LightManager(lightSources[i], boxes);
        }

        Entities
            .WithAll<OpaqueObject>()
            .WithoutBurst()
            .ForEach((in Box box, in Entity entity) => {
                foreach (var l in lightManagers.Values) {
                    l.Add(in box, in entity);
                }
            }).Run();
        
        foreach (var lightManager in lightManagers.Values) {
            lightManager.Compute();
        }
        
        lightSources.Dispose();
        lightSourceEntities.Dispose();
    }

    public IEnumerable<ShadowEdge> GetShadowEdgesForDebug() {
        foreach (var edge in GetShadowEdges()) {
            yield return edge;
        }
    }

    public NativeList<ShadowEdge> GetShadowEdges(Entity shadowSource) {
        return lightManagers[shadowSource].shadowEdges;
    }

    public NativeList<ShadowEdge> GetShadowEdges() {
        // TODO: For now, there is only one light manager
        foreach (var lightManager in lightManagers.Values) {
            return lightManager.shadowEdges;
        }
        throw new System.Exception();
    }

    private void Clear(Dictionary<Entity, LightManager> lightManagers) {
        foreach (var lightManager in lightManagers.Values) {
            lightManager.Dispose();
        }
        lightManagers.Clear();
    }

}

public class LightManager {
    private LightSource source;
    private float2 lightEdge1;
    private float2 lightEdge2;
    public NativeList<ShadowEdge> shadowEdges {get;}
    private NativeList<ProtoEdge> protoEdges;
    private NativeList<Entity> workingSet;
    private NativeHashMap<Entity, ShadowData> shadowData;
    private ComponentDataFromEntity<Box> boxes;

    private struct ShadowData {
        public Geometry.ShadowGeometry sg1;
        public Geometry.ShadowGeometry sg2;
    }

    private struct ProtoEdge : System.IComparable<ProtoEdge> {
        public float angle;
        public Entity opaque;

        public int CompareTo(ProtoEdge p) {
            return angle.CompareTo(p.angle);
        }
    }

    public LightManager(in LightSource source, ComponentDataFromEntity<Box> boxes) {
        this.source = source;
        lightEdge1 = source.GetMinEdgeNorm();
        lightEdge2 = source.GetMaxEdgeNorm();
        Debug.Assert(Lin.Cross(lightEdge1, lightEdge2) > 0);


        shadowEdges = new NativeList<ShadowEdge>(Allocator.TempJob);
        protoEdges = new NativeList<ProtoEdge>(Allocator.TempJob);
        workingSet = new NativeList<Entity>(Allocator.TempJob);
        shadowData = new NativeHashMap<Entity, ShadowData>(0, Allocator.TempJob);
        this.boxes = boxes;
    }

    public void Dispose() {
        shadowEdges.Dispose();
        protoEdges.Dispose();
        workingSet.Dispose();
        shadowData.Dispose();
    }

    public void Add(in Box box, in Entity entity) {
        float2 lightPos = source.pos;
        Geometry.CalculateShadowGeometry(box.ToRect(), lightPos, .05f, out var sg1, out var sg2);

        (float a1, float a2) = Angles(sg1.contact1, sg2.contact1);
        if (!math.isnan(a1)) {
            protoEdges.Add(new ProtoEdge{angle=a1, opaque=entity});
            protoEdges.Add(new ProtoEdge{angle=a2, opaque=entity});
            shadowData[entity] = new ShadowData{sg1 = sg1, sg2 = sg2};
        }
    }

    public void Compute() {
        protoEdges.Sort();
        workingSet.Clear();

        foreach (var protoEdge in protoEdges) {
            var scannedEdge = new Geometry.ShadowGeometry();

            bool addToWorking = true;
            {// Scanning
                var shadowDatum = shadowData[protoEdge.opaque];
                for (int i = 0; i < workingSet.Length; i++) {
                    if (workingSet[i] == protoEdge.opaque) {
                        workingSet.RemoveAtSwapBack(i);
                        addToWorking = false;
                        scannedEdge = shadowDatum.sg2;
                        break;
                    }
                }
                if (addToWorking) {
                    scannedEdge = shadowDatum.sg1;
                }
            }

            if (math.isfinite(protoEdge.angle)) {// Subtraction of working set from scannedEdge
                float scannedEdgeLength = 100;
                foreach (Entity entityToSubtract in workingSet) {
                    Box boxToSubtract = boxes[entityToSubtract];
                    Geometry.ShadowSubtract(
                        lightOrigin: source.pos,
                        shadowOrigin: scannedEdge.contact1,
                        shadowLength: ref scannedEdgeLength,
                        toSubtract: boxToSubtract
                    );
                }

                if (scannedEdgeLength > 0) {
                    shadowEdges.Add(new ShadowEdge {
                        contact1 = scannedEdge.contact1,
                        contact2 = scannedEdge.contact2,
                        opaque = protoEdge.opaque,
                        collider = Rect.FromLineSegment(scannedEdge.contact1, scannedEdge.contact1 + math.normalize(scannedEdge.contact1 - source.pos)*scannedEdgeLength, scannedEdge.id),
                        length = scannedEdgeLength,
                        leading = addToWorking,
                        lightSource = source.pos
                    });
                }
            }

            // This is here so that the owner of scannedEdge is not in the
            // working set during the subtraction calculations
            if (addToWorking) {
                workingSet.Add(protoEdge.opaque);
            }
        }
    }

    private float Angle(float2 point) {
        // The region the light shines on is the set of all points that are "in
        // front" of both lightEdge1 and lightEdge2
        float2 n = math.normalize(point - source.pos);
        // c1 > 0 iff n is in front of lightEdge1
        float c1 = Lin.Cross(lightEdge1, n);
        // c2 < 0 iff n is in front of lightEdge2
        float c2 = Lin.Cross(lightEdge2, n);
        if (c1 < 0 && c2 > 0) {
            return math.NAN;
        }
        if (c1 < 0) {
            return -math.INFINITY;
        }
        if (c2 > 0) {
            return math.INFINITY;
        }
        return 1 - math.dot(lightEdge1, n);
    }

    private (float, float) Angles(float2 p1, float2 p2) {
        float a1 = Angle(p1);
        float a2 = Angle(p2);
        if (a1 == math.NAN) {
            if (math.isfinite(a2)) {
                return (-math.INFINITY, a2);
            } else {
                return (math.NAN, math.NAN);
            }
        }
        if (a2 == math.NAN) {
            if (math.isfinite(a1)) {
                return (a1, math.INFINITY);
            } else {
                return (math.NAN, math.NAN);
            }
        }
        if (a1 >= a2) {
            return (math.NAN, math.NAN);
        }
        return (a1, a2);
    }
}

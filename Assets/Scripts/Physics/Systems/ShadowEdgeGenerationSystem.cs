using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;
using System.Collections;

using Physics.Math;
using UnityEngine;

using Rect = Physics.Math.Rect;

using ShadowEdge = ShadowEdgeGenerationSystem.ShadowEdge;

// Current plans for this class:
// Generate all manifolds representing contact between a shadow edge and a box, or a light edge and a box.
// These manifolds will be used to generate constraints elsewhere.
//
// Inputs to algorithm:
// - List of opaque boxes
// - List of light sources
// - List of shadowhitting boxes
//
// Global data structures:
// - box_to_manifold: multi map from shadowHit boxes to shadow edge manifolds involving them
// - initial_shadow_corner_manifolds: NativeList of shadow corner manifolds
// Steps of algorithm:
// 1. Ensure each light source has a corresponding LightManager, freshly initialized
// 2. For each light manager
//      2.a Take in all opaque boxes and shadowHitting boxes
//      2.b. Generate a list of (angle, shape) sorted by angle, where an entry
//          (a, s) means "'s' occupies the angular space starting at 'a', and
//          ending at the angle of the next entry"
//      2.c. Use the generated list to calculate manifolds for each shadowEdge, shadowHittingObj pair
//          Put these manifolds in box_to_manifold
//
//  3. For each box with manifolds m1, m2 ... mn
//      3.a. For each pair of manifolds mi, mj
//          3.a.a. If mi and mj intersect to create a shadow corner intersecting their box,
//              add the shadow corner to initial_shadow_corner_manifolds
//
//  4. For each light manager
//      4.a Mark any manifold (in box_to_manifold and
//      initial_shadow_corner_manifolds) illuminated by this light manager as
//      trash
//
//  5. Return all non-trash manifolds
//

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

    public struct ShadowEdgeManifold {
        public Geometry.Manifold manifold;
        public Entity box;
        public ShadowEdge shadowEdge;
    }

    Dictionary<Entity, LightManager> lightManagers;
    NativeList<ShadowEdgeManifold> shadowEdgeManifolds;

    protected override void OnCreate() {
        lightSourceQuery = GetEntityQuery(typeof(LightSource));
        lightManagers = new Dictionary<Entity, LightManager>();
        shadowEdgeManifolds = new NativeList<ShadowEdgeManifold>(Allocator.Persistent);
    }

    protected override void OnDestroy() {
        Clear(lightManagers);
        shadowEdgeManifolds.Dispose();
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

        /////////////////
        /// Temporary ///
        /////////////////
        // computing all the shadow edge manifolds. This algorithm is brute force

        shadowEdgeManifolds.Clear();

        foreach (var lightManager in lightManagers.Values) {
            Entities
                .WithAll<HitShadowsObject>()
                .WithoutBurst()
                .ForEach((in Box box, in Entity entity) => {
                    foreach (ShadowEdge edge in lightManager.shadowEdges) {
                        var manifoldNullable = Geometry.GetIntersectData(box.ToRect(), edge.collider);
                        if (manifoldNullable is Geometry.Manifold manifold) {
                            shadowEdgeManifolds.Add(new ShadowEdgeManifold{
                                box = entity,
                                manifold = manifold,
                                shadowEdge = edge
                            });
                        }
                    }
                }).Run();
        }

        /////////////////
        /////////////////
        /////////////////
        lightSources.Dispose();
        lightSourceEntities.Dispose();
    }

    public IEnumerable<ShadowEdge> GetShadowEdgesForDebug() {
        foreach (var lightManager in lightManagers.Values) {
            foreach (var edge in lightManager.shadowEdges) {
                yield return edge;
            }
        }
    }

    public NativeList<ShadowEdge> GetShadowEdges(Entity shadowSource) {
        return lightManagers[shadowSource].shadowEdges;
    }

    public NativeList<ShadowEdgeManifold> GetShadowEdgeManifolds() {
        return shadowEdgeManifolds;
    }

    private void Clear(Dictionary<Entity, LightManager> lightManagers) {
        foreach (var lightManager in lightManagers.Values) {
            lightManager.Dispose();
        }
        lightManagers.Clear();
    }

}

// TODO: figure out if it's worth it to make this a struct.
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
            shadowData.TryAdd(entity, new ShadowData{sg1 = sg1, sg2 = sg2});
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

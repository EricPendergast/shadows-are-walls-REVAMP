using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;
using System.Collections;

using Physics.Math;
using UnityEngine;

using Utilities;

using Rect = Physics.Math.Rect;

using ShadowEdgeManifold = ShadowEdgeGenerationSystem.ShadowEdgeManifold;

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
    private EntityQuery opaqueBoxesQuery;
    private EntityQuery shadHitBoxesQuery;

    public struct ShadowEdgeManifold {
        public Entity shadHitEntity;
        public Physics.Math.Geometry.Contact contact1;
        public Physics.Math.Geometry.Contact? contact2;

        public Entity castingEntity;
        public float2 mount1;
        public float2? mount2;

        public LightManager.ShapeType castingShapeType;

        public float overlap;
        public float2 normal;
        public float2 lightSource;
    }

    Dictionary<Entity, LightManager> lightManagers;
    NativeList<ShadowEdgeManifold> finalShadowEdgeManifolds;
    NativeMultiHashMap<Entity, ShadowEdgeManifold> boxManifolds;

    protected override void OnCreate() {
        lightSourceQuery = GetEntityQuery(typeof(LightSource));
        opaqueBoxesQuery = GetEntityQuery(typeof(Box), typeof(OpaqueObject));
        shadHitBoxesQuery = GetEntityQuery(typeof(Box), typeof(HitShadowsObject));
        lightManagers = new Dictionary<Entity, LightManager>();
        finalShadowEdgeManifolds = new NativeList<ShadowEdgeManifold>(Allocator.Persistent);
        boxManifolds = new NativeMultiHashMap<Entity, ShadowEdgeManifold>(0, Allocator.Persistent);
    }

    protected override void OnDestroy() {
        Clear(lightManagers);
        finalShadowEdgeManifolds.Dispose();
        boxManifolds.Dispose();
    }

    protected override void OnUpdate() {

        var lightSources = lightSourceQuery.ToComponentDataArray<LightSource>(Allocator.TempJob);
        var lightSourceEntities = lightSourceQuery.ToEntityArray(Allocator.TempJob);

        var opaqueBoxes = opaqueBoxesQuery.ToComponentDataArray<Box>(Allocator.TempJob);
        var opaqueBoxEntities = opaqueBoxesQuery.ToEntityArray(Allocator.TempJob);

        var shadHitBoxes = shadHitBoxesQuery.ToComponentDataArray<Box>(Allocator.TempJob);
        var shadHitBoxEntities = shadHitBoxesQuery.ToEntityArray(Allocator.TempJob);

        var boxes = GetComponentDataFromEntity<Box>(true);

        // Step 1: Initialization
        // Can be optimized
        Clear(lightManagers);
        for (int i = 0; i < lightSources.Length; i++) {
            lightManagers[lightSourceEntities[i]] = new LightManager(lightSources[i], lightSourceEntities[i], boxes);
        }


        boxManifolds.Clear();
        // Step 2: Computing initial contact manifolds
        foreach (var lm in lightManagers.Values) {
            // Compute the sorted shadow list
            // 2.a, 2.b
            lm.Precompute(opaqueBoxes, opaqueBoxEntities);
            // 2.c
            lm.ComputeInitialManifolds(shadHitBoxes, shadHitBoxEntities, ref boxManifolds);
        }

        var (boxesWithManifolds, length) = boxManifolds.GetUniqueKeyArray(Allocator.TempJob);
        // Step 3: Computing shadow corners
        // can optimize with IJobNativeMultiHashMapMergedSharedKeyIndices
        // TODO: Implement functions called here
        //for (int i = 0; i < length; i++) {
        //    Entity box = boxesWithManifolds[i];
        //    var manIt = It.Iterate(boxManifolds, box);
        //    while (manIt.MoveNext()) {
        //        var m1 = manIt.Current;
        //        var manIt2 = manIt;
        //        while (manIt2.MoveNext()) {
        //            var m2 = manIt2.Current;
        //            if (m1.Intersect(m2) is ShadowCornerManifold scm) {
        //                shadowCornerManifolds.Add(scm);
        //            }
        //        }
        //    }
        //}

        boxesWithManifolds.Dispose();

        // Step 4: Removing illuminated manifolds
        var illuminatedPoints = new NativeHashSet<float2>(0, Allocator.TempJob);

        // TODO: Implement functions called here
        //foreach (var lm in lightManagers.Values) {
        //    lm.MarkIlluminated(shadowCornerManifolds, illuminatedPoints);
        //    lm.MarkIlluminated(boxManifolds, illuminatedPoints);
        //}

        // Step 5: Store all non illuminated manifolds

        finalShadowEdgeManifolds.Clear();
        foreach (var kv in boxManifolds) {
            var seManifold = kv.Value;

            float2 c1 = seManifold.contact1.point;
            float2? c2Nullable = seManifold.contact2?.point;

            // Removing illuminated contact points
            if (illuminatedPoints.Contains(c1)) {
                if (c2Nullable is float2 c2 && !illuminatedPoints.Contains(c2)) {
                    seManifold.contact1 = (Geometry.Contact)seManifold.contact2;
                    seManifold.contact2 = null;
                } else {
                    continue;
                }
            } else {
                if (c2Nullable is float2 c2 && illuminatedPoints.Contains((float2)c2)) {
                    seManifold.contact2 = null;
                }
            }

            finalShadowEdgeManifolds.Add(seManifold);
        }

        // TODO: When implementing shadow corner collisions
        //finalShadowCornerManifolds.Clear();
        //foreach (var scManifold in shadowCornerManifolds) {
        //    if (!illuminatedPoints.Contains(scManifold.point)) {
        //        finalShadowCornerManifolds.Add(scManifold);
        //    }
        //}

        illuminatedPoints.Dispose();
        lightSources.Dispose();
        lightSourceEntities.Dispose();
        opaqueBoxes.Dispose();
        opaqueBoxEntities.Dispose();
        shadHitBoxes.Dispose();
        shadHitBoxEntities.Dispose();
    }

    public IEnumerable<LightManager.ShadowEdgeDebugInfo> GetShadowEdgesForDebug() {
        foreach (var lightManager in lightManagers.Values) {
            foreach (var edge in lightManager.IterShadowEdgeDebugInfo()) {
                yield return edge;
            }
        }
    }

    public IEnumerable<float2> GetRenderPoints(Entity lightSource) {
        return lightManagers[lightSource].GetRenderPoints();
    }

    public NativeList<ShadowEdgeManifold> GetShadowEdgeManifolds() {
        return finalShadowEdgeManifolds;
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
    private Entity sourceEntity;
    private float2 lightEdge1;
    private float2 lightEdge2;
    private NativeList<OpaqueSection> sortedShadows;
    private NativeList<ProtoEdge> protoEdges;
    private NativeList<Entity> workingSet;
    private NativeHashMap<Entity, ShadowData> shadowData;
    private ComponentDataFromEntity<Box> boxes;

    private struct ShadowData {
        public Geometry.ShadowGeometry sg1;
        public Geometry.ShadowGeometry sg2;
    }

    public enum ShapeType {
        Box, Light
    }

    // An opaue section is an angular range in the light source occupied by an
    // opaque shape (or empty). An empty section is indicated by setting
    // shapeType=Light. An OpaqueSection contains the shadow edge at the start
    // of its section.
    // OTHER EXPLANATION:
    // There is one opaque section per shadow edge. The opaque sections are in
    // a list, sorted by angle. An opaque section contains info about what
    // occupies the space between its shadow edge and then next section's.
    private struct OpaqueSection : System.IComparable<OpaqueSection> {
        public float angle;

        public Entity castingShape;
        public ShapeType castingShapeType;

        public float edgeStart;
        public float edgeEnd;

        // Where the shadow edge is mounted on the casting shape
        public float2 mount1;
        public float2? mount2;
        public int edgeId;

        public Entity edgeOwner;
        public ShapeType edgeOwnerType;

        public int CompareTo(OpaqueSection other) {
            return angle.CompareTo(other.angle);
        }

        public float2 ShadowEndPoint(float2 lightPos) {
            return lightPos + math.normalize(mount1 - lightPos)*edgeEnd;
        }
    }

    private struct ProtoEdge : System.IComparable<ProtoEdge> {
        public float angle;
        public Entity opaque;
    
        public int CompareTo(ProtoEdge p) {
            return angle.CompareTo(p.angle);
        }
    }
    
    public LightManager(in LightSource source, in Entity sourceEntity, ComponentDataFromEntity<Box> boxes) {
        this.source = source;
        this.sourceEntity = sourceEntity;
        lightEdge1 = source.GetMinEdgeNorm();
        lightEdge2 = source.GetMaxEdgeNorm();
        Debug.Assert(Lin.Cross(lightEdge1, lightEdge2) > 0);


        protoEdges = new NativeList<ProtoEdge>(Allocator.TempJob);
        sortedShadows = new NativeList<OpaqueSection>(Allocator.TempJob);
        workingSet = new NativeList<Entity>(Allocator.TempJob);
        shadowData = new NativeHashMap<Entity, ShadowData>(0, Allocator.TempJob);
        this.boxes = boxes;
    }

    public void Dispose() {
        protoEdges.Dispose();
        sortedShadows.Dispose();
        workingSet.Dispose();
        shadowData.Dispose();
    }

    public void Precompute(NativeArray<Box> opaqueBoxes, NativeArray<Entity> opaqueBoxEntities) {
        Debug.Assert(opaqueBoxes.Length == opaqueBoxEntities.Length);

        for (int i = 0; i < opaqueBoxes.Length; i++) {
            Geometry.CalculateShadowGeometry(opaqueBoxes[i].ToRect(), source.pos, .05f, out var sg1, out var sg2);

            (float a1, float a2) = Angles(sg1.contact1, sg2.contact1);
            if (!math.isnan(a1)) {
                Entity opaqueEntity = opaqueBoxEntities[i];
                // TODO: I think there are some unecessary steps here. Can't I just put sg1 and sg2 into protoEdges?
                protoEdges.Add(new ProtoEdge{angle=a1, opaque=opaqueEntity});
                protoEdges.Add(new ProtoEdge{angle=a2, opaque=opaqueEntity});
                shadowData.TryAdd(opaqueEntity, new ShadowData{sg1 = sg1, sg2 = sg2});
            }
        }

        Compute();
    }

    public void ComputeInitialManifolds(NativeArray<Box> shadHitBoxes, NativeArray<Entity> shadHitBoxEntities, ref NativeMultiHashMap<Entity, ShadowEdgeManifold> shadowEdgeManifolds) {
        Debug.Assert(shadHitBoxes.Length == shadHitBoxEntities.Length);
        for (int i = 0; i < shadHitBoxes.Length; i++) {
            var shBox = shadHitBoxes[i];
            var shEntity = shadHitBoxEntities[i];
            foreach (OpaqueSection sect in sortedShadows) {
                var manifoldNullable = Geometry.GetIntersectData(shBox.ToRect(), Rect.FromLineSegment(sect.mount1, sect.ShadowEndPoint(source.pos), sect.edgeId));
                if (manifoldNullable is Geometry.Manifold manifold) {
                    shadowEdgeManifolds.Add(shEntity, new ShadowEdgeManifold{
                        contact1 = manifold.contact1,
                        contact2 = manifold.contact2,
                        mount1 = sect.mount1,
                        mount2 = sect.mount2,
                        shadHitEntity = shEntity,
                        castingEntity = sect.edgeOwner,
                        castingShapeType = sect.edgeOwnerType,
                        overlap = manifold.overlap,
                        lightSource = source.pos,
                        normal = manifold.normal
                    });
                }
            }
        }
    }

    private void Compute() {
        protoEdges.Sort();
        workingSet.Clear();

        foreach (var protoEdge in protoEdges) {
            var scannedEdge = new Geometry.ShadowGeometry();

            bool leading = true;
            {// Scanning
                var shadowDatum = shadowData[protoEdge.opaque];
                for (int i = 0; i < workingSet.Length; i++) {
                    if (workingSet[i] == protoEdge.opaque) {
                        workingSet.RemoveAtSwapBack(i);
                        leading = false;
                        scannedEdge = shadowDatum.sg2;
                        break;
                    }
                }
                if (leading) {
                    scannedEdge = shadowDatum.sg1;
                }
            }

            if (math.isfinite(protoEdge.angle)) {// Subtraction of working set from scannedEdge
                float scannedEdgeLength = 100;
                // Contains the entity of the box with the most overlap with the shadow edge
                Entity subtracted = sourceEntity;
                foreach (Entity entityToSubtract in workingSet) {
                    Box boxToSubtract = boxes[entityToSubtract];
                    float prevLength = scannedEdgeLength;
                    Geometry.ShadowSubtract(
                        lightOrigin: source.pos,
                        shadowOrigin: scannedEdge.contact1,
                        shadowLength: ref scannedEdgeLength,
                        toSubtract: boxToSubtract
                    );
                    if (prevLength != scannedEdgeLength) {
                        subtracted = entityToSubtract;
                    }
                }


                if (scannedEdgeLength > 0) {
                    ShapeType subtractedType = subtracted == sourceEntity ? ShapeType.Light : ShapeType.Box;

                    sortedShadows.Add(new OpaqueSection {
                        angle = protoEdge.angle,
                        // If this is the trailing edge, then the shape
                        // occupying the range is the one after the edge, which
                        // is the one which subtracted the largest amount from
                        // it.
                        castingShape = leading ? protoEdge.opaque : subtracted,
                        castingShapeType = leading ? ShapeType.Box : subtractedType,
                        edgeStart = math.distance(source.pos, scannedEdge.contact1),
                        edgeEnd = math.distance(source.pos, scannedEdge.contact1) + scannedEdgeLength,
                        edgeOwner = protoEdge.opaque,
                        mount1 = scannedEdge.contact1,
                        mount2 = scannedEdge.contact2,
                        edgeId = new int2(scannedEdge.id, source.id).GetHashCode()
                    });
                }
            }

            // This is here so that the owner of scannedEdge is not in the
            // working set during the subtraction calculations
            if (leading) {
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

    public struct ShadowEdgeDebugInfo {
        public float2 mount1;
        public float2? mount2;
        public float2 endpoint;
        public int id;
    }

    public IEnumerable<ShadowEdgeDebugInfo> IterShadowEdgeDebugInfo() {
        foreach (var section in sortedShadows) {
            yield return new ShadowEdgeDebugInfo {
                mount1 = section.mount1,
                mount2 = section.mount2,
                endpoint = section.ShadowEndPoint(source.pos),
                id = section.edgeId
            };
        }
    }

    public IEnumerable<float2> GetRenderPoints() {
        foreach (var section in sortedShadows) {
            float2 shadowBegin = section.mount1;
            float2 shadowEnd = section.ShadowEndPoint(source.pos);

            if (section.edgeOwner == section.castingShape) {
                yield return shadowEnd;
                yield return shadowBegin;
            } else {
                yield return shadowBegin;
                yield return shadowEnd;
            }
        }
    }
}

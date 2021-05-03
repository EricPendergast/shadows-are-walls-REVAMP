using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;
using System.Collections;

using Physics.Math;
using UnityEngine;

using System.Runtime.InteropServices;

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

using LightManager = LightManagerNew;

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

        public ShapeType castingShapeType;

        public float overlap;
        public float2 normal;
        public float2 lightSource;
    }

    Dictionary<Entity, LightManager> lightManagers;
    NativeList<ShadowEdgeManifold> finalShadowEdgeManifolds;
    NativeList<ShadowEdgeManifold> finalLightEdgeManifolds;
    NativeMultiHashMap<Entity, ShadowEdgeManifold> boxManifolds;

    protected override void OnCreate() {
        lightSourceQuery = GetEntityQuery(typeof(LightSource));
        opaqueBoxesQuery = GetEntityQuery(typeof(Box), typeof(OpaqueObject));
        shadHitBoxesQuery = GetEntityQuery(typeof(Box), typeof(HitShadowsObject));
        lightManagers = new Dictionary<Entity, LightManager>();
        finalShadowEdgeManifolds = new NativeList<ShadowEdgeManifold>(Allocator.Persistent);
        finalLightEdgeManifolds = new NativeList<ShadowEdgeManifold>(Allocator.Persistent);
        boxManifolds = new NativeMultiHashMap<Entity, ShadowEdgeManifold>(0, Allocator.Persistent);
    }

    protected override void OnDestroy() {
        Clear(lightManagers);
        finalShadowEdgeManifolds.Dispose();
        finalLightEdgeManifolds.Dispose();
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
            lightManagers[lightSourceEntities[i]] = new LightManager(lightSources[i], lightSourceEntities[i]);
        }


        boxManifolds.Clear();
        // Step 2: Computing initial contact manifolds
        foreach (var lm in lightManagers.Values) {
            // Compute the sorted shadow list
            // 2.a, 2.b, 2.c
            lm.ComputeManifolds(
                opaqueBoxes, opaqueBoxEntities,
                shadHitBoxes, shadHitBoxEntities,
                ref boxManifolds);
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
        finalLightEdgeManifolds.Clear();
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

            switch (seManifold.castingShapeType) {
                case ShapeType.Box:
                    finalShadowEdgeManifolds.Add(seManifold);
                    break;
                case ShapeType.Light:
                    finalLightEdgeManifolds.Add(seManifold);
                    break;
            }
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

    //public IEnumerable<float2> GetRenderPoints(Entity lightSource) {
    //    return lightManagers[lightSource].GetRenderPoints();
    //}

    public NativeList<ShadowEdgeManifold> GetShadowEdgeManifolds() {
        return finalShadowEdgeManifolds;
    }

    public NativeList<ShadowEdgeManifold> GetLightEdgeManifolds() {
        return finalLightEdgeManifolds;
    }

    private void Clear(Dictionary<Entity, LightManager> lightManagers) {
        foreach (var lightManager in lightManagers.Values) {
            lightManager.Dispose();
        }
        lightManagers.Clear();
    }

}

public enum ShapeType {
    Box, Light
}

// TODO: figure out if it's worth it to make this a struct.
public class LightManagerOld {
    private LightSource source;
    private Entity sourceEntity;
    private float2 leadingLightEdge;
    private float2 trailingLightEdge;
    private NativeList<OpaqueSection> sortedShadows;
    private NativeList<ProtoEdge> protoEdges;
    private NativeList<Entity> workingSet;
    private ComponentDataFromEntity<Box> boxes;


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

        // The shape that occupies (is fully exposed to light in) this section.
        public Entity occupyingShape;
        public ShapeType occupyingShapeType;

        public float2 edgeDir;
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
            return lightPos + edgeDir*edgeEnd;
        }
    }

    private struct ProtoEdge : System.IComparable<ProtoEdge> {
        public float angle;
        public Entity source;
        public Geometry.ShadowGeometry sg;
    
        public int CompareTo(ProtoEdge p) {
            return angle.CompareTo(p.angle);
        }
    }
    
    public LightManagerOld(in LightSource source, in Entity sourceEntity, ComponentDataFromEntity<Box> boxes) {
        this.source = source;
        this.sourceEntity = sourceEntity;
        leadingLightEdge = source.GetLeadingEdgeNorm();
        trailingLightEdge = source.GetTrailingEdgeNorm();
        Debug.Assert(Lin.Cross(leadingLightEdge, trailingLightEdge) > 0);


        protoEdges = new NativeList<ProtoEdge>(Allocator.TempJob);
        sortedShadows = new NativeList<OpaqueSection>(Allocator.TempJob);
        workingSet = new NativeList<Entity>(Allocator.TempJob);
        this.boxes = boxes;
    }

    public void Dispose() {
        protoEdges.Dispose();
        sortedShadows.Dispose();
        workingSet.Dispose();
    }

    public void Precompute(NativeArray<Box> opaqueBoxes, NativeArray<Entity> opaqueBoxEntities) {
        Debug.Assert(opaqueBoxes.Length == opaqueBoxEntities.Length);

        for (int i = 0; i < opaqueBoxes.Length; i++) {
            Geometry.CalculateShadowGeometry(opaqueBoxes[i].ToRect(), source.pos, .05f, out var sg1, out var sg2);

            (float a1, float a2) = Angles(sg1.contact1, sg2.contact1);
            if (!math.isnan(a1)) {
                Entity opaqueEntity = opaqueBoxEntities[i];
                protoEdges.Add(new ProtoEdge{angle=a1, source=opaqueEntity, sg=sg1});
                protoEdges.Add(new ProtoEdge{angle=a2, source=opaqueEntity, sg=sg2});
            }
        }

        protoEdges.Add(new ProtoEdge{angle=0, source=sourceEntity, sg=new Geometry.ShadowGeometry{contact1=source.pos, contact2=null, id=source.minEdgeId}});
        protoEdges.Add(new ProtoEdge{angle=RawAngleOfNormal(trailingLightEdge), source=sourceEntity, sg=new Geometry.ShadowGeometry{contact1=source.pos, contact2=null, id=source.maxEdgeId}});

        ComputeSortedShadows();
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

    private void ComputeSortedShadows() {
        protoEdges.Sort();
        workingSet.Clear();

        foreach (var protoEdge in protoEdges) {

            bool leading = true;
            {// Scanning
                for (int i = 0; i < workingSet.Length; i++) {
                    if (workingSet[i] == protoEdge.source) {
                        workingSet.RemoveAtSwapBack(i);
                        leading = false;
                        break;
                    }
                }
            }

            AddOpaqueSection(protoEdge, leading);

            // This is here so that the owner of protoEdge is not in the
            // working set during the subtraction calculations
            if (leading) {
                workingSet.Add(protoEdge.source);
            }
        }

    }

    // nextShape is the entity of the shape which overlapped the most with the
    // edge. It is called nextShape because it is the shape right after the
    // shadow edge.
    private void SubtractWorkingSetFromEdge(float2 edgeDir, float edgeStart, ref float edgeEnd, in Rect edgeCastingShape, out Entity nextShape) {
        nextShape = sourceEntity;
        foreach (Entity entityToSubtract in workingSet) {
            if (entityToSubtract == sourceEntity) {
                continue;
            }
            Box boxToSubtract = boxes[entityToSubtract];
            float prevLength = edgeEnd;

            Geometry.ShadowSubtract(
                lightOrigin: source.pos,
                shadowDirection: edgeDir,
                shadowStart: edgeStart,
                shadowEnd: ref edgeEnd,
                //shadowCastingShape: edgeCastingShape,
                toSubtract: boxToSubtract.ToRect()
            );
            if (prevLength != edgeEnd) {
                nextShape = entityToSubtract;
            }
        }
    }
    // Adds an OpaqueSection to sortedShadows, corresponding to protoEdge.
    // Subtracts workingSet from the resulting shadow edge
    private void AddOpaqueSection(ProtoEdge protoEdge, bool leading) {
        bool isLightEdge = protoEdge.source == sourceEntity;

        if (math.isfinite(protoEdge.angle)) {
            float edgeEnd = 100;
            
            float edgeStart;
            float2 edgeDir;
            Rect edgeCastingShape;

            if (isLightEdge) {
                edgeStart = 0;
                edgeDir = leading ? leadingLightEdge : trailingLightEdge;
                edgeCastingShape = new Rect(source.pos, float2.zero, float2.zero, 0);
            } else {
                edgeStart = math.distance(source.pos, protoEdge.sg.contact1);
                edgeDir = (protoEdge.sg.contact1 - source.pos)/edgeStart;
                edgeCastingShape = boxes[protoEdge.source].ToRect();
            }

            SubtractWorkingSetFromEdge(
                edgeDir: edgeDir,
                edgeStart: edgeStart,
                edgeEnd: ref edgeEnd,
                edgeCastingShape: edgeCastingShape,
                nextShape: out var nextShape
            );

            if (isLightEdge && !leading) {
                nextShape = sourceEntity;
            }

            if (edgeEnd >= edgeStart) {
                ShapeType nextShapeType = nextShape == sourceEntity ? ShapeType.Light : ShapeType.Box;

                sortedShadows.Add(new OpaqueSection {
                    angle = protoEdge.angle,
                    // If this is the trailing edge, then the shape
                    // occupying the range is the one after the edge, which
                    // is the one which subtracted the largest amount from
                    // it.
                    occupyingShape = leading ? protoEdge.source : nextShape,
                    occupyingShapeType = leading ? ShapeType.Box : nextShapeType,
                    edgeDir = edgeDir,
                    edgeStart = edgeStart,
                    edgeEnd = edgeEnd,
                    edgeOwner = protoEdge.source,
                    edgeOwnerType = isLightEdge ? ShapeType.Light : ShapeType.Box,
                    mount1 = protoEdge.sg.contact1,
                    mount2 = protoEdge.sg.contact2,
                    edgeId = protoEdge.sg.id
                });
            }
        }
    }

    private float RawAngleOfNormal(float2 normal) {
        return 1 - math.dot(leadingLightEdge, normal);
    }

    private float Angle(float2 point) {
        // The region the light shines on is the set of all points that are "in
        // front" of both leadingLightEdge and trailingLightEdge
        float2 n = math.normalize(point - source.pos);
        // c1 > 0 iff n is in front of leadingLightEdge
        float c1 = Lin.Cross(leadingLightEdge, n);
        // c2 < 0 iff n is in front of trailingLightEdge
        float c2 = Lin.Cross(trailingLightEdge, n);
        if (c1 < 0 && c2 > 0) {
            return math.NAN;
        }
        if (c1 < 0) {
            return -math.INFINITY;
        }
        if (c2 > 0) {
            return math.INFINITY;
        }
        return RawAngleOfNormal(n);
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
        yield return source.pos;
        foreach (var section in sortedShadows) {
            float2 shadowBegin = section.mount1;
            float2 shadowEnd = section.ShadowEndPoint(source.pos);

            if (section.edgeOwnerType == ShapeType.Light) {
                yield return shadowEnd;
            } else {
                if (section.edgeOwner == section.occupyingShape) {
                    yield return shadowEnd;
                    yield return source.pos;

                    yield return shadowBegin;
                } else {
                    yield return shadowBegin;
                    yield return source.pos;

                    yield return shadowEnd;
                }
            }
        }
    }
}

public class LightManagerNew {
    private LightSource source;
    private Entity sourceEntity;
    private float2 leadingLightEdge;
    private float2 trailingLightEdge;
    private NativeList<ShapeEdge> shapeEdges;
    private NativeList<ShapeEdge.OpaqueData> opaqueWorkingSet;
    private NativeList<ShapeEdge.ShadHitData> shadHitWorkingSet;

    private List<ShadowEdgeDebugInfo> shadowEdgeDebugInfo;
    //private ComponentDataFromEntity<Box> boxes;

    [StructLayout(LayoutKind.Explicit)]
    private struct ShapeEdge : System.IComparable<ShapeEdge> {
        public enum Owner : byte {
            Light,
            Opaque,
            ShadHit
        }

        [FieldOffset(0)]
        public Owner type;
        
        public struct LightData {
            public float angle;
            public float2 direction;
            public int id;
        }
        [FieldOffset(5)]
        public LightData lightData;
        
        public struct OpaqueData {
            public float angle;
            public Entity source;
            public Rect rect;
            public float2 mount1;
            public float2? mount2;
            public int id;
        }
        [FieldOffset(5)]
        public OpaqueData opaqueData;

        public struct ShadHitData {
            public float angle;
            public Entity source;
            public Rect rect;
        }
        [FieldOffset(5)]
        public ShadHitData shadHitData;

        public static implicit operator ShapeEdge(LightData d) {
            return new ShapeEdge {
                type = Owner.Light,
                lightData = d
            };
        }

        public static implicit operator ShapeEdge(OpaqueData d) {
            return new ShapeEdge {
                type = Owner.Opaque,
                opaqueData = d
            };
        }

        public static implicit operator ShapeEdge(ShadHitData d) {
            return new ShapeEdge {
                type = Owner.ShadHit,
                shadHitData = d
            };
        }

        private float GetAngle() {
            // TODO: Is struct layout guaranteed?
            switch (type) {
                case Owner.Light:
                    return lightData.angle;
                case Owner.ShadHit:
                    return lightData.angle;
                case Owner.Opaque:
                    return lightData.angle;
                default:
                    return -1234;
            }
        }

        public int CompareTo(ShapeEdge other) {
            return GetAngle().CompareTo(other.GetAngle());
        }
    }

    public LightManagerNew(in LightSource source, in Entity sourceEntity/*, ComponentDataFromEntity<Box> boxes*/) {
        this.source = source;
        this.sourceEntity = sourceEntity;
        leadingLightEdge = source.GetLeadingEdgeNorm();
        trailingLightEdge = source.GetTrailingEdgeNorm();
        Debug.Assert(Lin.Cross(leadingLightEdge, trailingLightEdge) > 0);

        shapeEdges = new NativeList<ShapeEdge>(Allocator.TempJob);
        opaqueWorkingSet = new NativeList<ShapeEdge.OpaqueData>(Allocator.TempJob);
        shadHitWorkingSet = new NativeList<ShapeEdge.ShadHitData>(Allocator.TempJob);

        shadowEdgeDebugInfo = new List<ShadowEdgeDebugInfo>();
        //this.boxes = boxes;
    }

    public void Dispose() {
        shapeEdges.Dispose();
        opaqueWorkingSet.Dispose();
        shadHitWorkingSet.Dispose();
    }

    public void ComputeManifolds(
            NativeArray<Box> opaqueBoxes, NativeArray<Entity> opaqueBoxEntities, 
            NativeArray<Box> shadHitBoxes, NativeArray<Entity> shadHitBoxEntities,
            ref NativeMultiHashMap<Entity, ShadowEdgeManifold> shadowEdgeManifolds) {

        // FOR DEBUG
        shadowEdgeDebugInfo.Clear();

        StoreShapeEdges(opaqueBoxes, opaqueBoxEntities, shadHitBoxes, shadHitBoxEntities);
        shapeEdges.Sort();

        foreach (ShapeEdge edge in shapeEdges) {
            if (edge.type == ShapeEdge.Owner.Light) {
                HandleLightEdge(in edge.lightData, ref shadowEdgeManifolds);
            } else if (edge.type == ShapeEdge.Owner.Opaque) {
                HandleOpaqueEdge(edge.opaqueData, ref shadowEdgeManifolds);
            } else if (edge.type == ShapeEdge.Owner.ShadHit) {
                if (!TryRemove(ref shadHitWorkingSet, edge.shadHitData.source)) {
                    shadHitWorkingSet.Add(edge.shadHitData);
                }
            }
        }
    }

    private void SubtractWorkingSetFromEdge(float2 edgeDir, float edgeStart, ref float edgeEnd) {
        foreach (ShapeEdge.OpaqueData opaqueEdge in opaqueWorkingSet) {
            Geometry.ShadowSubtract(
                lightOrigin: source.pos,
                shadowDirection: edgeDir,
                shadowStart: edgeStart,
                shadowEnd: ref edgeEnd,
                toSubtract: opaqueEdge.rect
            );
        }
    }

    private void HandleLightEdge(in ShapeEdge.LightData lightEdge, ref NativeMultiHashMap<Entity, ShadowEdgeManifold> shadowEdgeManifolds) {
        float edgeStart = 0;
        float edgeEnd = 100;
        float2 edgeDir = lightEdge.direction;

        SubtractWorkingSetFromEdge(
            edgeDir: edgeDir,
            edgeStart: edgeStart,
            edgeEnd: ref edgeEnd
        );

        if (edgeEnd > edgeStart) {
            float2 startPoint = source.pos + edgeDir*edgeStart;
            float2 endPoint = source.pos + edgeDir*edgeEnd;

            // FOR DEBUG //
            shadowEdgeDebugInfo.Add(new ShadowEdgeDebugInfo{endpoint = endPoint, id = lightEdge.id, mount1 = startPoint, mount2 = null});
            ///////////////

            foreach (ShapeEdge.ShadHitData shadHitObject in shadHitWorkingSet) {
                var manifoldNullable = Geometry.GetIntersectData(
                    shadHitObject.rect,
                    Rect.FromLineSegment(
                        startPoint,
                        endPoint,
                        lightEdge.id
                    )
                );

                if (manifoldNullable is Geometry.Manifold manifold) {
                    shadowEdgeManifolds.Add(shadHitObject.source, new ShadowEdgeManifold{
                        contact1 = manifold.contact1,
                        contact2 = manifold.contact2,
                        mount1 = source.pos,
                        mount2 = null,
                        shadHitEntity = shadHitObject.source,
                        castingEntity = sourceEntity,
                        castingShapeType = ShapeType.Light,
                        overlap = manifold.overlap,
                        lightSource = source.pos,
                        normal = manifold.normal
                    });
                }
            }
        }
    }

    private void HandleOpaqueEdge(in ShapeEdge.OpaqueData opaqueEdge, ref NativeMultiHashMap<Entity, ShadowEdgeManifold> shadowEdgeManifolds) {
        bool removed = TryRemove(ref opaqueWorkingSet, opaqueEdge.source);

        if (math.isfinite(opaqueEdge.angle)) {
            float edgeStart = math.distance(source.pos, opaqueEdge.mount1);
            float edgeEnd = 100;
            float2 edgeDir = (opaqueEdge.mount1 - source.pos)/edgeStart;

            SubtractWorkingSetFromEdge(
                edgeDir: edgeDir,
                edgeStart: edgeStart,
                edgeEnd: ref edgeEnd
            );

            if (edgeEnd > edgeStart) {
                float2 startPoint = source.pos + edgeDir*edgeStart;
                float2 endPoint = source.pos + edgeDir*edgeEnd;

                // FOR DEBUG //
                shadowEdgeDebugInfo.Add(new ShadowEdgeDebugInfo{endpoint = endPoint, id = opaqueEdge.id, mount1 = opaqueEdge.mount1, mount2 = opaqueEdge.mount2});
                ///////////////

                foreach (ShapeEdge.ShadHitData shadHitObject in shadHitWorkingSet) {
                    var manifoldNullable = Geometry.GetIntersectData(
                        shadHitObject.rect,
                        Rect.FromLineSegment(
                            startPoint,
                            endPoint,
                            opaqueEdge.id
                        )
                    );

                    if (manifoldNullable is Geometry.Manifold manifold) {
                        shadowEdgeManifolds.Add(shadHitObject.source, new ShadowEdgeManifold{
                            contact1 = manifold.contact1,
                            contact2 = manifold.contact2,
                            mount1 = opaqueEdge.mount1,
                            mount2 = opaqueEdge.mount2,
                            shadHitEntity = shadHitObject.source,
                            castingEntity = opaqueEdge.source,
                            castingShapeType = ShapeType.Box,
                            overlap = manifold.overlap,
                            lightSource = source.pos,
                            normal = manifold.normal
                        });
                    }
                }
            }
        }

        if (!removed) {
            opaqueWorkingSet.Add(opaqueEdge);
        }
    }

    public void StoreShapeEdges(
            NativeArray<Box> opaqueBoxes, NativeArray<Entity> opaqueBoxEntities, 
            NativeArray<Box> shadHitBoxes, NativeArray<Entity> shadHitBoxEntities) {
        Debug.Assert(opaqueBoxes.Length == opaqueBoxEntities.Length);
        Debug.Assert(shadHitBoxes.Length == shadHitBoxEntities.Length);
        
        for (int i = 0; i < opaqueBoxes.Length; i++) {
            Rect rect = opaqueBoxes[i].ToRect();
            Geometry.CalculateShadowGeometry(rect, source.pos, .05f, out var sg1, out var sg2);

            (float a1, float a2) = Angles(sg1.contact1, sg2.contact1);
            if (!math.isnan(a1)) {
                Entity opaqueEntity = opaqueBoxEntities[i];
                shapeEdges.Add(new ShapeEdge.OpaqueData{
                    angle = a1, 
                    source = opaqueEntity,
                    mount1 = sg1.contact1,
                    mount2 = sg1.contact2,
                    id = sg1.id,
                    rect = rect
                });
                shapeEdges.Add(new ShapeEdge.OpaqueData{
                    angle = a2, 
                    source = opaqueEntity,
                    mount1 = sg2.contact1,
                    mount2 = sg2.contact2,
                    id = sg2.id,
                    rect = rect
                });
            }
        }

        for (int i = 0; i < shadHitBoxes.Length; i++) {
            Rect rect = shadHitBoxes[i].ToRect();
            Geometry.CalculateShadowGeometry(rect, source.pos, .05f, out var sg1, out var sg2);

            (float a1, float a2) = Angles(sg1.contact1, sg2.contact1);
            if (!math.isnan(a1)) {
                Entity shadHitEntity = shadHitBoxEntities[i];
                shapeEdges.Add(new ShapeEdge.ShadHitData{
                    angle = a1, 
                    source = shadHitEntity,
                    rect = rect
                });
                shapeEdges.Add(new ShapeEdge.ShadHitData{
                    angle = a2, 
                    source = shadHitEntity,
                    rect = rect
                });
            }
        }

        shapeEdges.Add(new ShapeEdge.LightData{
            angle = 0, 
            direction = source.GetLeadingEdgeNorm(),
            id = source.minEdgeId
        });
        shapeEdges.Add(new ShapeEdge.LightData{
            angle = RawAngleOfNormal(trailingLightEdge),
            direction = source.GetTrailingEdgeNorm(),
            id = source.maxEdgeId
        });
    }

    // Yes this function and the other TryRemove are exactly the same (except a
    // type). But I think there is no generic way to do this. Unless I do
    // interfaces, but that seems more complicated than its worth.
    private static bool TryRemove(ref NativeList<ShapeEdge.OpaqueData> workingSet, Entity opaque) {
        bool removed = false;
        for (int i = 0; i < workingSet.Length; i++) {
            if (workingSet[i].source == opaque) {
                workingSet.RemoveAtSwapBack(i);
                removed = true;
                break;
            }
        }

        return removed;
    }

    private static bool TryRemove(ref NativeList<ShapeEdge.ShadHitData> workingSet, Entity opaque) {
        bool removed = false;
        for (int i = 0; i < workingSet.Length; i++) {
            if (workingSet[i].source == opaque) {
                workingSet.RemoveAtSwapBack(i);
                removed = true;
                break;
            }
        }

        return removed;
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

    private float Angle(float2 point) {
        // The region the light shines on is the set of all points that are "in
        // front" of both leadingLightEdge and trailingLightEdge
        float2 n = math.normalize(point - source.pos);
        // c1 > 0 iff n is in front of leadingLightEdge
        float c1 = Lin.Cross(leadingLightEdge, n);
        // c2 < 0 iff n is in front of trailingLightEdge
        float c2 = Lin.Cross(trailingLightEdge, n);
        if (c1 < 0 && c2 > 0) {
            return math.NAN;
        }
        if (c1 < 0) {
            return -math.INFINITY;
        }
        if (c2 > 0) {
            return math.INFINITY;
        }
        return RawAngleOfNormal(n);
    }

    private float RawAngleOfNormal(float2 normal) {
        return 1 - math.dot(leadingLightEdge, normal);
    }

    public struct ShadowEdgeDebugInfo {
        public float2 mount1;
        public float2? mount2;
        public float2 endpoint;
        public int id;
    }

    public IEnumerable<ShadowEdgeDebugInfo> IterShadowEdgeDebugInfo() {
        return this.shadowEdgeDebugInfo;
    }
}

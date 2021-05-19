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
    // TODO: Replace this with boxOverlappingEdges
    NativeMultiHashMap<Entity, ShadowEdgeManifold> boxManifolds;

    // TODO: This will replace some of the above stuff
    NativeMultiHashMap<Entity, CornerCalculator.Edge> boxOverlappingEdges;
    NativeList<LightSource> lightSources;
    NativeList<LightManager.AngleCalculator> lightAngleCalculators;

    protected override void OnCreate() {
        lightSourceQuery = GetEntityQuery(typeof(LightSource));
        opaqueBoxesQuery = GetEntityQuery(typeof(Box), typeof(OpaqueObject));
        shadHitBoxesQuery = GetEntityQuery(typeof(Box), typeof(HitShadowsObject));
        lightManagers = new Dictionary<Entity, LightManager>();
        finalShadowEdgeManifolds = new NativeList<ShadowEdgeManifold>(Allocator.Persistent);
        finalLightEdgeManifolds = new NativeList<ShadowEdgeManifold>(Allocator.Persistent);
        boxManifolds = new NativeMultiHashMap<Entity, ShadowEdgeManifold>(0, Allocator.Persistent);

        boxOverlappingEdges = new NativeMultiHashMap<Entity, CornerCalculator.Edge>(0, Allocator.Persistent);
        lightSources = new NativeList<LightSource>(Allocator.Persistent);
        lightAngleCalculators = new NativeList<LightManager.AngleCalculator>(Allocator.Persistent);
    }

    protected override void OnDestroy() {
        Clear(lightManagers);
        finalShadowEdgeManifolds.Dispose();
        finalLightEdgeManifolds.Dispose();
        boxManifolds.Dispose();

        boxOverlappingEdges.Dispose();
        lightSources.Dispose();
        lightAngleCalculators.Clear();
    }

    protected override void OnUpdate() {
        var lightSources = this.lightSources;
        var lightAngleCalculators = this.lightAngleCalculators;

        lightSources.Clear();
        lightAngleCalculators.Clear();
        Entities.ForEach((in LightSource lightSource) => {
            lightSources.Add(lightSource);
            lightAngleCalculators.Add(new LightManager.AngleCalculator(lightSource));
        }).Run();

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
            lightManagers[lightSourceEntities[i]] = new LightManager(lightSources[i], lightSourceEntities[i], i);
        }


        boxManifolds.Clear();
        boxOverlappingEdges.Clear();
        // Step 2: Computing initial contact manifolds
        foreach (var lm in lightManagers.Values) {
            // Compute the sorted shadow list
            // 2.a, 2.b, 2.c
            lm.ComputeManifolds(
                opaqueBoxes, opaqueBoxEntities,
                shadHitBoxes, shadHitBoxEntities,
                ref boxManifolds,
                ref boxOverlappingEdges);
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

    public List<CornerCalculator.Corner> GetShadowIslandsForDebug() {
        var ret = new List<CornerCalculator.Corner>();
        Entities.WithAll<Box, HitShadowsObject>()
            .WithoutBurst()
            .ForEach((in Box box, in Entity entity) => {
                var cc = new CornerCalculator(
                    box,
                    lightSources,
                    lightAngleCalculators,
                    It.Iterate(boxOverlappingEdges, entity)
                );

                foreach (var item in cc.GetIslandsForDebug()) {
                    ret.Add(item);
                }
            }).Run();
        return ret;
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

public class LightManagerNew {
    private LightSource source;
    private AngleCalculator angleCalc;
    private Entity sourceEntity;
    private int sourceIndex;
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
            public float2 mount;
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

    public LightManagerNew(in LightSource source, in Entity sourceEntity, int sourceIndex) {
        this.source = source;
        this.sourceEntity = sourceEntity;
        angleCalc = new AngleCalculator(source);

        shapeEdges = new NativeList<ShapeEdge>(Allocator.TempJob);
        opaqueWorkingSet = new NativeList<ShapeEdge.OpaqueData>(Allocator.TempJob);
        shadHitWorkingSet = new NativeList<ShapeEdge.ShadHitData>(Allocator.TempJob);

        shadowEdgeDebugInfo = new List<ShadowEdgeDebugInfo>();

        this.sourceIndex = sourceIndex;
    }

    public void Dispose() {
        shapeEdges.Dispose();
        opaqueWorkingSet.Dispose();
        shadHitWorkingSet.Dispose();
    }

    public void ComputeManifolds(
            NativeArray<Box> opaqueBoxes, NativeArray<Entity> opaqueBoxEntities, 
            NativeArray<Box> shadHitBoxes, NativeArray<Entity> shadHitBoxEntities,
            // TODO: Remove
            ref NativeMultiHashMap<Entity, ShadowEdgeManifold> shadowEdgeManifolds,
            ref NativeMultiHashMap<Entity, CornerCalculator.Edge> boxOverlappingEdges) {

        // FOR DEBUG
        shadowEdgeDebugInfo.Clear();

        StoreShapeEdges(opaqueBoxes, opaqueBoxEntities, shadHitBoxes, shadHitBoxEntities);
        shapeEdges.Sort();

        foreach (ShapeEdge edge in shapeEdges) {
            if (edge.type == ShapeEdge.Owner.Light) {
                HandleLightEdge(in edge.lightData, ref shadowEdgeManifolds, ref boxOverlappingEdges);
            } else if (edge.type == ShapeEdge.Owner.Opaque) {
                HandleOpaqueEdge(edge.opaqueData, ref shadowEdgeManifolds, ref boxOverlappingEdges);
            } else if (edge.type == ShapeEdge.Owner.ShadHit) {
                HandleShadHitEdge(in edge.shadHitData, ref shadowEdgeManifolds, ref boxOverlappingEdges);
            }
        }
    }

    private void SubtractWorkingSetFromEdge(float2 edgeDir, float edgeStart, ref float edgeEnd) {
        foreach (ShapeEdge.OpaqueData opaqueEdge in opaqueWorkingSet) {
            // TODO:
            // if this edge is leading and close to opaqueEdge's leading, skip
            // if this edge is trailing and close to opaqueEdge's trailing, skip
            

            Geometry.ShadowSubtract(
                lightOrigin: source.pos,
                shadowDirection: edgeDir,
                shadowStart: edgeStart,
                shadowEnd: ref edgeEnd,
                toSubtract: opaqueEdge.rect
            );
        }
    }

    private void HandleLightEdge(in ShapeEdge.LightData lightEdge, ref NativeMultiHashMap<Entity, ShadowEdgeManifold> shadowEdgeManifolds, ref NativeMultiHashMap<Entity, CornerCalculator.Edge> boxOverlappingEdges) {
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
                    Debug.Assert(lightEdge.angle == angleCalc.MinAngle() || lightEdge.angle == angleCalc.MaxAngle());
                    boxOverlappingEdges.Add(shadHitObject.source, new CornerCalculator.Edge{
                        angle = lightEdge.angle,
                        direction = lightEdge.direction,
                        lightSource = sourceIndex,
                        type = CornerCalculator.Edge.Type.edge,
                        lightSide = lightEdge.angle == angleCalc.MinAngle() ? 1 : -1
                    });
                }
            }
        }
    }

    private void HandleOpaqueEdge(in ShapeEdge.OpaqueData opaqueEdge, ref NativeMultiHashMap<Entity, ShadowEdgeManifold> shadowEdgeManifolds, ref NativeMultiHashMap<Entity, CornerCalculator.Edge> boxOverlappingEdges) {
        bool removed = TryRemove(ref opaqueWorkingSet, opaqueEdge.source);
        bool leading = !removed;

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

                    var manifoldNullable = Geometry.GetShadowEdgeIntersectData(
                        shadHitRect: shadHitObject.rect,
                        edgeStart: startPoint,
                        edgeEnd: endPoint,
                        edgeIsLeading: leading,
                        edgeId: opaqueEdge.id
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
                        boxOverlappingEdges.Add(shadHitObject.source, new CornerCalculator.Edge{
                            angle = opaqueEdge.angle,
                            direction = edgeDir,
                            lightSource = sourceIndex,
                            type = CornerCalculator.Edge.Type.edge,
                            lightSide = leading ? -1 : 1
                        });
                    }
                }
            }
        }

        if (!removed) {
            opaqueWorkingSet.Add(opaqueEdge);
        }
    }

    private void HandleShadHitEdge(in ShapeEdge.ShadHitData shadHitEdge, ref NativeMultiHashMap<Entity, ShadowEdgeManifold> shadowEdgeManifolds, ref NativeMultiHashMap<Entity, CornerCalculator.Edge> boxOverlappingEdges) {
        bool removed = TryRemove(ref shadHitWorkingSet, shadHitEdge.source);
        if (!removed) {
            shadHitWorkingSet.Add(shadHitEdge);
        }

        bool leading = !removed;

        float edgeStart = math.distance(source.pos, shadHitEdge.mount);
        float edgeEnd = 100;
        float2 edgeDir = (shadHitEdge.mount - source.pos)/edgeStart;

        SubtractWorkingSetFromEdge(
            edgeDir: edgeDir,
            edgeStart: edgeStart,
            edgeEnd: ref edgeEnd
        );

        // If the mount of this edge is illuminated, add a distant shadow edge
        // with its light side toward this edge of the shadow hitting object.
        if (edgeEnd > edgeStart) {
            boxOverlappingEdges.Add(shadHitEdge.source, new CornerCalculator.Edge{
                angle = leading ? -math.INFINITY : math.INFINITY,
                direction = edgeDir,
                lightSource = sourceIndex,
                type = CornerCalculator.Edge.Type.edge,
                lightSide = leading ? 1 : -1
            });
        }
    }

    public void StoreShapeEdges(
            NativeArray<Box> opaqueBoxes, NativeArray<Entity> opaqueBoxEntities, 
            NativeArray<Box> shadHitBoxes, NativeArray<Entity> shadHitBoxEntities) {
        Debug.Assert(opaqueBoxes.Length == opaqueBoxEntities.Length);
        Debug.Assert(shadHitBoxes.Length == shadHitBoxEntities.Length);
        
        for (int i = 0; i < opaqueBoxes.Length; i++) {
            Rect rect = opaqueBoxes[i].ToRect();
            Geometry.CalculateShadowGeometry(rect, source.pos, .005f, out var sg1, out var sg2);

            (float a1, float a2) = angleCalc.Angles(sg1.contact1, sg2.contact1);
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
            Geometry.CalculateShadowGeometry(rect, source.pos, .005f, out var sg1, out var sg2);

            (float a1, float a2) = angleCalc.Angles(sg1.contact1, sg2.contact1);
            if (!math.isnan(a1)) {
                Entity shadHitEntity = shadHitBoxEntities[i];
                shapeEdges.Add(new ShapeEdge.ShadHitData{
                    angle = a1, 
                    source = shadHitEntity,
                    rect = rect,
                    mount = sg1.contact1
                });
                shapeEdges.Add(new ShapeEdge.ShadHitData{
                    angle = a2, 
                    source = shadHitEntity,
                    rect = rect,
                    mount = sg2.contact1
                });
            }
        }

        shapeEdges.Add(new ShapeEdge.LightData{
            angle = angleCalc.MinAngle(), 
            direction = source.GetLeadingEdgeNorm(),
            id = source.minEdgeId
        });
        shapeEdges.Add(new ShapeEdge.LightData{
            angle = angleCalc.MaxAngle(),
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


    public struct AngleCalculator {
        private float2 sourcePos;
        private float2 leadingLightEdge;
        private float2 trailingLightEdge;

        public AngleCalculator(LightSource source) {
            this.sourcePos = source.pos;
            this.leadingLightEdge = source.GetLeadingEdgeNorm();
            this.trailingLightEdge = source.GetTrailingEdgeNorm();
            Debug.Assert(Lin.Cross(leadingLightEdge, trailingLightEdge) > 0);
        }

        public (float, float) Angles(float2 p1, float2 p2) {
            float a1 = Angle(p1);
            float a2 = Angle(p2);

            bool swap = Lin.Cross(p1 - sourcePos, p2 - sourcePos) < 0;

            if (swap) {
                var tmp = a1;
                a1 = a2;
                a2 = tmp;
            }

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

            return swap ? (a2, a1) : (a1, a2);
        }

        public float Angle(float2 point) {
            // The region the light shines on is the set of all points that are "in
            // front" of both leadingLightEdge and trailingLightEdge
            float2 n = math.normalize(point - sourcePos);
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

        public float RawAngleOfNormal(float2 normal) {
            return 1 - math.dot(leadingLightEdge, normal);
        }

        public float MinAngle() {
            return 0;
        }
        public float MaxAngle() {
            return RawAngleOfNormal(trailingLightEdge);
        }
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

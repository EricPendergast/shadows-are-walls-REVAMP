using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using Physics.Math;
using UnityEngine;

using System.Runtime.InteropServices;

using Rect = Physics.Math.Rect;

using ShadowEdgeManifold = ShadowEdgeGenerationSystem.ShadowEdgeManifold;
using EdgeMountsMap = Unity.Collections.NativeMultiHashMap<CornerCalculator.Edge.EdgeKey, CornerCalculator.EdgeMount>;
using EdgeMount = CornerCalculator.EdgeMount;

using FloatPair = System.ValueTuple<float, float>;

public class ShadowEdgeCalculator {
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
            // TODO: This maybe should be 2 objects. So it would just have mount and id
            public float2 mount1;
            public int id1;
            public float2? mount2;
            public int? id2;
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

    public ShadowEdgeCalculator(in LightSource source, in Entity sourceEntity, int sourceIndex) {
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

    // TODO: Rename to something better. (This does not compute the manifolds anymore)
    public void ComputeManifolds(
            NativeArray<Box> opaqueBoxes, NativeArray<Entity> opaqueBoxEntities, 
            NativeArray<Box> shadHitBoxes, NativeArray<Entity> shadHitBoxEntities,
            ref NativeMultiHashMap<Entity, CornerCalculator.Edge> boxOverlappingEdges,
            ref EdgeMountsMap edgeMounts) {

        // FOR DEBUG
        shadowEdgeDebugInfo.Clear();

        StoreShapeEdges(opaqueBoxes, opaqueBoxEntities, shadHitBoxes, shadHitBoxEntities);
        shapeEdges.Sort();

        foreach (ShapeEdge edge in shapeEdges) {
            if (edge.type == ShapeEdge.Owner.Light) {
                HandleLightEdge(in edge.lightData, ref boxOverlappingEdges, ref edgeMounts);
            } else if (edge.type == ShapeEdge.Owner.Opaque) {
                HandleOpaqueEdge(edge.opaqueData, ref boxOverlappingEdges, ref edgeMounts);
            } else if (edge.type == ShapeEdge.Owner.ShadHit) {
                HandleShadHitEdge(in edge.shadHitData, ref boxOverlappingEdges);
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

    private void HandleLightEdge(in ShapeEdge.LightData lightEdge, ref NativeMultiHashMap<Entity, CornerCalculator.Edge> boxOverlappingEdges, ref EdgeMountsMap edgeMounts) {
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
            shadowEdgeDebugInfo.Add(new ShadowEdgeDebugInfo{endpoint = endPoint, id1 = lightEdge.id, id2 = null, mount1 = startPoint, mount2 = null});
            ///////////////

            bool addedToOverlapping = false;

            var edge = new CornerCalculator.Edge{
                angle = lightEdge.angle,
                direction = lightEdge.direction,
                lightSource = sourceIndex,
                lightSide = lightEdge.angle == angleCalc.MinAngle() ? (sbyte)1 : (sbyte)-1,
            };

            foreach (ShapeEdge.ShadHitData shadHitObject in shadHitWorkingSet) {
                bool intersecting = Geometry.IsIntersectingShadowEdge(
                    rect: shadHitObject.rect,
                    lightOrigin: source.pos,
                    shadowDirection: lightEdge.direction,
                    shadowStart: edgeStart,
                    shadowEnd: edgeEnd
                );

                if (intersecting) {
                    Debug.Assert(lightEdge.angle == angleCalc.MinAngle() || lightEdge.angle == angleCalc.MaxAngle());
                    boxOverlappingEdges.Add(shadHitObject.source, edge);
                    addedToOverlapping = true;
                }

            }
            // Only add the edge mounts if there is an object which will need
            // them.
            if (addedToOverlapping) {
                edgeMounts.Add(edge.GetEdgeKey(), new EdgeMount {
                    castingEntity = sourceEntity,
                    castingShapeType = ShapeType.Light,
                    point = source.pos,
                    shapeCenter = source.pos,
                    id = lightEdge.id,
                });
            }
        }
    }

    private void HandleOpaqueEdge(in ShapeEdge.OpaqueData opaqueEdge, ref NativeMultiHashMap<Entity, CornerCalculator.Edge> boxOverlappingEdges, ref EdgeMountsMap edgeMounts) {
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
                shadowEdgeDebugInfo.Add(new ShadowEdgeDebugInfo{endpoint = endPoint, id1 = opaqueEdge.id1, id2 = opaqueEdge.id2, mount1 = opaqueEdge.mount1, mount2 = opaqueEdge.mount2});
                ///////////////

                bool addedToOverlapping = false;

                var edge = new CornerCalculator.Edge{
                    angle = opaqueEdge.angle,
                    direction = edgeDir,
                    lightSource = sourceIndex,
                    lightSide = (sbyte)(leading ? -1 : 1),
                };

                foreach (ShapeEdge.ShadHitData shadHitObject in shadHitWorkingSet) {
                    bool intersecting = Geometry.IsIntersectingShadowEdge(
                        rect: shadHitObject.rect,
                        lightOrigin: source.pos,
                        shadowDirection: edgeDir,
                        shadowStart: edgeStart,
                        shadowEnd: edgeEnd
                    );

                    if (intersecting) {
                        boxOverlappingEdges.Add(shadHitObject.source, edge);
                        addedToOverlapping = true;
                    }
                }

                // Only add the edge mounts if there is an object which will need
                // them.
                if (addedToOverlapping) {
                    edgeMounts.Add(edge.GetEdgeKey(), new EdgeMount{
                        castingEntity = opaqueEdge.source,
                        castingShapeType = ShapeType.Box,
                        point = opaqueEdge.mount1,
                        shapeCenter = opaqueEdge.rect.pos,
                        id = opaqueEdge.id1
                    });
                    if (opaqueEdge.mount2 is float2 mount) {
                        edgeMounts.Add(edge.GetEdgeKey(), new EdgeMount{
                            castingEntity = opaqueEdge.source,
                            castingShapeType = ShapeType.Box,
                            point = mount,
                            shapeCenter = opaqueEdge.rect.pos,
                            id = opaqueEdge.id2.Value
                        });
                    }
                }
            }
        }

        if (!removed) {
            opaqueWorkingSet.Add(opaqueEdge);
        }
    }

    private void HandleShadHitEdge(in ShapeEdge.ShadHitData shadHitEdge, ref NativeMultiHashMap<Entity, CornerCalculator.Edge> boxOverlappingEdges) {
        bool removed = TryRemove(ref shadHitWorkingSet, shadHitEdge.source);
        if (!removed) {
            shadHitWorkingSet.Add(shadHitEdge);
        }

        // Do an illumination check if the edge is in the range of the light
        if (math.isfinite(shadHitEdge.angle)) {
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
                    lightSide = (sbyte)(leading ? 1 : -1),
                });
            }
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

            var fp = angleCalc.Angles(sg1.contact1, sg2.contact1);
            float a1 = fp.Item1;
            float a2 = fp.Item2;

            if (!math.isnan(a1)) {
                Entity opaqueEntity = opaqueBoxEntities[i];
                shapeEdges.Add(new ShapeEdge.OpaqueData{
                    angle = a1, 
                    source = opaqueEntity,
                    mount1 = sg1.contact1,
                    mount2 = sg1.contact2,
                    id1 = sg1.id1,
                    id2 = sg1.id2,
                    rect = rect
                });
                shapeEdges.Add(new ShapeEdge.OpaqueData{
                    angle = a2, 
                    source = opaqueEntity,
                    mount1 = sg2.contact1,
                    mount2 = sg2.contact2,
                    id1 = sg2.id1,
                    id2 = sg2.id2,
                    rect = rect
                });
            }
        }

        for (int i = 0; i < shadHitBoxes.Length; i++) {
            Rect rect = shadHitBoxes[i].ToRect();
            Geometry.CalculateShadowGeometry(rect, source.pos, .005f, out var sg1, out var sg2);

            var fp = angleCalc.Angles(sg1.contact1, sg2.contact1);
            float a1 = fp.Item1;
            float a2 = fp.Item2;
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

    public struct ShadowEdgeDebugInfo {
        public float2 mount1;
        public float2? mount2;
        public float2 endpoint;
        public int id1;
        public int? id2;
    }

    public IEnumerable<ShadowEdgeDebugInfo> IterShadowEdgeDebugInfo() {
        return this.shadowEdgeDebugInfo;
    }
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

    public FloatPair Angles(float2 p1, float2 p2) {
        float a1 = Angle(p1);
        float a2 = Angle(p2);

        bool swap = Lin.Cross(p1 - sourcePos, p2 - sourcePos) < 0;

        if (swap) {
            var tmp = a1;
            a1 = a2;
            a2 = tmp;
        }

        if (math.isnan(a1)) {
            if (math.isfinite(a2)) {
                return new FloatPair(-math.INFINITY, a2);
            } else {
                return new FloatPair(math.NAN, math.NAN);
            }
        }
        if (math.isnan(a2)) {
            if (math.isfinite(a1)) {
                return new FloatPair(a1, math.INFINITY);
            } else {
                return new FloatPair(math.NAN, math.NAN);
            }
        }

        return swap ? new FloatPair(a2, a1) : new FloatPair(a1, a2);
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

    // Gives the cost of rotating an edge with given direction to the point.
    // Positive angle rotation gives a positive cost.
    public float CostOfRotationTo(float2 edgeDirection, float2 point) {
        float2 n = math.normalize(point - sourcePos);
        return Lin.Cross(edgeDirection, n);
    }

    // Gives edgeDirection rotated by 90 degrees towards its illuminated side
    public float2 NormalTowardsLight(float2 edgeDirection, int lightDirection) {
        return Lin.Cross(lightDirection, edgeDirection);
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

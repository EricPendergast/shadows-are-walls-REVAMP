using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Burst;
using System.Collections.Generic;

using Physics.Math;
using UnityEngine;

using System.Runtime.InteropServices;

using Rect = Physics.Math.Rect;

using EdgeMountsMap = Unity.Collections.NativeMultiHashMap<ShadowCornerCalculator.Edge.EdgeKey, ShadowCornerCalculator.EdgeMount>;
using EdgeMount = ShadowCornerCalculator.EdgeMount;

public class ShadowEdgeCalculatorClassWrapper {
    public ShadowEdgeCalculator shadowEdgeCalculator;
    public ShadowEdgeCalculatorClassWrapper(in LightSource source, in Entity sourceEntity, in AngleCalculator angleCalc, int sourceIndex) {
        shadowEdgeCalculator = new ShadowEdgeCalculator(in source, in sourceEntity, in angleCalc, sourceIndex);
    }
    public void Dispose() {
        shadowEdgeCalculator.Dispose();
    }
}

public struct FloatPair {
    public float Item1;
    public float Item2;
    public FloatPair(float Item1, float Item2) {
        this.Item1 = Item1;
        this.Item2 = Item2;
    }
    public void Deconstruct(out float Item1, out float Item2) {
        Item1 = this.Item1;
        Item2 = this.Item2;
    }
}

public struct ShadowEdgeCalculator {
    public struct Emitter {
        public NativeMultiHashMap<Entity, ShadowCornerCalculator.Edge>? boxOverlappingEdges;
        public EdgeMountsMap? edgeMounts;
        public static List<ShadowEdgeDebugInfo> shadowEdgeDebugInfo;

        [BurstDiscard]
        public void DebugEmit(ShadowEdgeDebugInfo i) {
            if (shadowEdgeDebugInfo != null) {
                shadowEdgeDebugInfo.Add(i);
            }
        }

        public void EmitOverlappingEdge(Entity e, ShadowCornerCalculator.Edge edge) {
            if (boxOverlappingEdges != null) {
                boxOverlappingEdges.Value.Add(e, edge);
            }
        }

        public void EmitEdgeMount(ShadowCornerCalculator.Edge.EdgeKey edgeKey, ShadowCornerCalculator.EdgeMount edgeMount) {
            if (edgeMounts != null) {
                edgeMounts.Value.Add(edgeKey, edgeMount);
            }
        }
    }

    public struct Env {
        public NativeArray<Box> opaqueBoxes;
        public NativeArray<Position> opaquePositions;
        public NativeArray<Entity> opaqueBoxEntities;
        public NativeArray<Box> shadHitBoxes;
        public NativeArray<Position> shadHitPositions;
        public NativeArray<Entity> shadHitBoxEntities;

        public void Dispose() {
            opaqueBoxes.Dispose();
            opaquePositions.Dispose();
            opaqueBoxEntities.Dispose();
            shadHitBoxes.Dispose();
            shadHitPositions.Dispose();
            shadHitBoxEntities.Dispose();
        }
    }

    private AngleCalculator angleCalc;
    private float2 sourcePos {get => angleCalc.SourcePos;}
    private LightSource lightSource;
    private Entity sourceEntity;
    private int sourceIndex;
    private NativeList<ShapeEdge> shapeEdges;
    private NativeList<ShapeEdge.OpaqueData> opaqueWorkingSet;
    private NativeList<ShapeEdge.ShadHitData> shadHitWorkingSet;


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

    public ShadowEdgeCalculator(in LightSource source, in Entity sourceEntity, in AngleCalculator angleCalc, int sourceIndex) {
        this.lightSource = source;
        this.sourceEntity = sourceEntity;
        this.angleCalc = angleCalc;

        shapeEdges = new NativeList<ShapeEdge>(Allocator.TempJob);
        opaqueWorkingSet = new NativeList<ShapeEdge.OpaqueData>(Allocator.TempJob);
        shadHitWorkingSet = new NativeList<ShapeEdge.ShadHitData>(Allocator.TempJob);

        this.sourceIndex = sourceIndex;
    }

    public void Dispose() {
        shapeEdges.Dispose();
        opaqueWorkingSet.Dispose();
        shadHitWorkingSet.Dispose();
    }

    private void Reset() {
        shapeEdges.Clear();
        opaqueWorkingSet.Clear();
        shadHitWorkingSet.Clear();
    }

    // TODO: Rename to something better. (This does not compute the manifolds anymore)
    public void ComputeShadowEdgeContacts(Env env, Emitter emitter) {

        Reset();

        StoreShapeEdges(env);
        shapeEdges.Sort();

        foreach (ShapeEdge edge in shapeEdges) {
            if (edge.type == ShapeEdge.Owner.Light) {
                HandleLightEdge(in edge.lightData, emitter);
            } else if (edge.type == ShapeEdge.Owner.Opaque) {
                HandleOpaqueEdge(edge.opaqueData, emitter);
            } else if (edge.type == ShapeEdge.Owner.ShadHit) {
                HandleShadHitEdge(in edge.shadHitData, emitter);
            }
        }
    }

    private void SubtractWorkingSetFromEdge(float2 edgeDir, float edgeStart, ref float edgeEnd) {
        foreach (ShapeEdge.OpaqueData opaqueEdge in opaqueWorkingSet) {
            // TODO:
            // if this edge is leading and close to opaqueEdge's leading, skip
            // if this edge is trailing and close to opaqueEdge's trailing, skip
            

            Geometry.ShadowSubtract(
                lightOrigin: sourcePos,
                shadowDirection: edgeDir,
                shadowStart: edgeStart,
                shadowEnd: ref edgeEnd,
                toSubtract: opaqueEdge.rect
            );
        }
    }

    private void HandleLightEdge(in ShapeEdge.LightData lightEdge, Emitter emitter) {
        float edgeStart = 0;
        float edgeEnd = 100;
        float2 edgeDir = lightEdge.direction;

        SubtractWorkingSetFromEdge(
            edgeDir: edgeDir,
            edgeStart: edgeStart,
            edgeEnd: ref edgeEnd
        );

        if (edgeEnd > edgeStart) {
            float2 startPoint = sourcePos + edgeDir*edgeStart;
            float2 endPoint = sourcePos + edgeDir*edgeEnd;

            emitter.DebugEmit(new ShadowEdgeDebugInfo{endpoint = endPoint, id1 = lightEdge.id, id2 = null, mount1 = startPoint, mount2 = null});

            bool addedToOverlapping = false;

            var edge = new ShadowCornerCalculator.Edge{
                angle = lightEdge.angle,
                direction = lightEdge.direction,
                lightSource = sourceIndex,
                lightSide = lightEdge.angle == angleCalc.MinAngle() ? (sbyte)1 : (sbyte)-1,
            };

            foreach (ShapeEdge.ShadHitData shadHitObject in shadHitWorkingSet) {
                bool intersecting = Geometry.IsIntersectingShadowEdge(
                    rect: shadHitObject.rect,
                    lightOrigin: sourcePos,
                    shadowDirection: lightEdge.direction,
                    shadowStart: edgeStart,
                    shadowEnd: edgeEnd,
                    correctPartialIntersection: false
                );

                if (intersecting) {
                    Debug.Assert(lightEdge.angle == angleCalc.MinAngle() || lightEdge.angle == angleCalc.MaxAngle());
                    emitter.EmitOverlappingEdge(shadHitObject.source, edge);
                    addedToOverlapping = true;
                }

            }
            // Only add the edge mounts if there is an object which will need
            // them.
            if (addedToOverlapping) {
                emitter.EmitEdgeMount(edge.GetEdgeKey(), new EdgeMount {
                    castingEntity = sourceEntity,
                    castingShapeType = EdgeSourceType.Light,
                    point = sourcePos,
                    shapeCenter = sourcePos,
                    id = lightEdge.id,
                });
            }
        }
    }

    private void HandleOpaqueEdge(in ShapeEdge.OpaqueData opaqueEdge, Emitter emitter) {
        bool removed = TryRemove(ref opaqueWorkingSet, opaqueEdge.source);
        bool leading = !removed;

        if (math.isfinite(opaqueEdge.angle)) {
            float edgeStart = math.distance(sourcePos, opaqueEdge.mount1);
            float edgeEnd = 100;
            float2 edgeDir = (opaqueEdge.mount1 - sourcePos)/edgeStart;

            SubtractWorkingSetFromEdge(
                edgeDir: edgeDir,
                edgeStart: edgeStart,
                edgeEnd: ref edgeEnd
            );

            if (edgeEnd > edgeStart) {
                float2 startPoint = sourcePos + edgeDir*edgeStart;
                float2 endPoint = sourcePos + edgeDir*edgeEnd;

                // FOR DEBUG //
                emitter.DebugEmit(new ShadowEdgeDebugInfo{endpoint = endPoint, id1 = opaqueEdge.id1, id2 = opaqueEdge.id2, mount1 = opaqueEdge.mount1, mount2 = opaqueEdge.mount2});
                ///////////////

                bool addedToOverlapping = false;

                var edge = new ShadowCornerCalculator.Edge{
                    angle = opaqueEdge.angle,
                    direction = edgeDir,
                    lightSource = sourceIndex,
                    lightSide = (sbyte)(leading ? -1 : 1),
                };

                foreach (ShapeEdge.ShadHitData shadHitObject in shadHitWorkingSet) {
                    bool intersecting = Geometry.IsIntersectingShadowEdge(
                        rect: shadHitObject.rect,
                        lightOrigin: sourcePos,
                        shadowDirection: edgeDir,
                        shadowStart: edgeStart,
                        shadowEnd: edgeEnd,
                        correctPartialIntersection: true
                    );

                    if (intersecting) {
                        emitter.EmitOverlappingEdge(shadHitObject.source, edge);
                        addedToOverlapping = true;
                    }
                }

                // Only add the edge mounts if there is an object which will need
                // them.
                if (addedToOverlapping) {
                    emitter.EmitEdgeMount(edge.GetEdgeKey(), new EdgeMount{
                        castingEntity = opaqueEdge.source,
                        castingShapeType = EdgeSourceType.Box,
                        point = opaqueEdge.mount1,
                        shapeCenter = opaqueEdge.rect.pos,
                        id = opaqueEdge.id1
                    });
                    if (opaqueEdge.mount2 is float2 mount) {
                        emitter.EmitEdgeMount(edge.GetEdgeKey(), new EdgeMount{
                            castingEntity = opaqueEdge.source,
                            castingShapeType = EdgeSourceType.Box,
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

    private void HandleShadHitEdge(in ShapeEdge.ShadHitData shadHitEdge, Emitter emitter) {
        bool removed = TryRemove(ref shadHitWorkingSet, shadHitEdge.source);
        if (!removed) {
            shadHitWorkingSet.Add(shadHitEdge);
        }

        // Do an illumination check if the edge is in the range of the light
        if (math.isfinite(shadHitEdge.angle)) {
            bool leading = !removed;

            float edgeStart = math.distance(sourcePos, shadHitEdge.mount);
            float edgeEnd = 100;
            float2 edgeDir = (shadHitEdge.mount - sourcePos)/edgeStart;

            SubtractWorkingSetFromEdge(
                edgeDir: edgeDir,
                edgeStart: edgeStart,
                edgeEnd: ref edgeEnd
            );

            // If this edge is illuminated, add an illumination tag for it.
            if (edgeEnd > edgeStart) {
                ShadowCornerCalculator.Edge edge;
                if (leading) {
                    edge = new ShadowCornerCalculator.Edge{
                        isIlluminationTag = true,
                        angle = -math.INFINITY,
                        direction = edgeDir,
                        lightSource = sourceIndex,
                        lightSide = 1,
                    };
                } else {
                    edge = new ShadowCornerCalculator.Edge{
                        isIlluminationTag = true,
                        angle = math.INFINITY,
                        direction = edgeDir,
                        lightSource = sourceIndex,
                        lightSide = -1,
                    };
                }
                emitter.EmitOverlappingEdge(shadHitEdge.source, edge);
                emitter.EmitEdgeMount(edge.GetEdgeKey(), new ShadowCornerCalculator.EdgeMount{
                    castingEntity = sourceEntity,
                    castingShapeType = EdgeSourceType.Light,
                    id =  new int2(lightSource.id, edge.lightSide).GetHashCode(),
                    point = sourcePos,
                    shapeCenter = sourcePos
                });
            }
        }
    }

    private void StoreShapeEdges(Env env) {

        Debug.Assert(env.opaqueBoxes.Length == env.opaqueBoxEntities.Length);
        Debug.Assert(env.shadHitBoxes.Length == env.shadHitBoxEntities.Length);
        
        for (int i = 0; i < env.opaqueBoxes.Length; i++) {
            Rect rect = env.opaqueBoxes[i].ToRect(env.opaquePositions[i]);
            Geometry.CalculateShadowGeometry(rect, sourcePos, .005f, out var sg1, out var sg2);

            var fp = angleCalc.Angles(sg1.contact1, sg2.contact1);
            float a1 = fp.Item1;
            float a2 = fp.Item2;

            if (!math.isnan(a1)) {
                Entity opaqueEntity = env.opaqueBoxEntities[i];
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

        for (int i = 0; i < env.shadHitBoxes.Length; i++) {
            Rect rect = env.shadHitBoxes[i].ToRect(env.shadHitPositions[i]);
            if (rect.Contains(sourcePos)) {
                Entity shadHitEntity = env.shadHitBoxEntities[i];
                shapeEdges.Add(new ShapeEdge.ShadHitData{
                    angle = -math.INFINITY, 
                    source = shadHitEntity,
                    rect = rect,
                    // Mount not used since angle is infinite
                    mount = math.NAN
                });
                shapeEdges.Add(new ShapeEdge.ShadHitData{
                    angle = math.INFINITY, 
                    source = shadHitEntity,
                    rect = rect,
                    // Mount not used since angle is infinite
                    mount = math.NAN
                });
                continue;
            }
            Geometry.CalculateShadowGeometry(rect, sourcePos, .005f, out var sg1, out var sg2);

            var fp = angleCalc.Angles(sg1.contact1, sg2.contact1);
            float a1 = fp.Item1;
            float a2 = fp.Item2;
            if (!math.isnan(a1)) {
                Entity shadHitEntity = env.shadHitBoxEntities[i];
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
            direction = angleCalc.MinDirection(),
            id = lightSource.minEdgeId
        });
        shapeEdges.Add(new ShapeEdge.LightData{
            angle = angleCalc.MaxAngle(),
            direction = angleCalc.MaxDirection(),
            id = lightSource.maxEdgeId
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
}

public struct AngleCalculator {
    private float2 sourcePos;
    private float2 leadingLightEdge;
    private float2 trailingLightEdge;

    public AngleCalculator(LightSource source, Position pos) : 
        this(
            pos.pos,
            source.GetLeadingEdgeNorm(pos.rot),
            source.GetTrailingEdgeNorm(pos.rot)) {}

    public AngleCalculator(float2 sourcePos, float2 edgeDir1, float2 edgeDir2) {
        this.sourcePos = sourcePos;
        if (Lin.Cross(edgeDir1, edgeDir2) > 0) {
            this.leadingLightEdge = edgeDir1;
            this.trailingLightEdge = edgeDir2;
        } else {
            this.leadingLightEdge = edgeDir2;
            this.trailingLightEdge = edgeDir1;
        }
    }

    public float2 SourcePos {get => sourcePos;}

    public FloatPair Angles(float2 p1, float2 p2) {
        float a1 = Angle(p1);
        float a2 = Angle(p2);

        // Swap a1 and a2 so that the counterclockwise distance between them is
        // smaller than the clockwise distance. We remember that we swapped so
        // we can swap them back at the end.
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

        if (math.isinf(a1) && math.isinf(a2) && a1 > a2) {
            return new FloatPair(math.NAN, math.NAN);
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

    public float2 MinDirection() {
        return leadingLightEdge;
    }

    public float2 MaxDirection() {
        return trailingLightEdge;
    }

    public float MinAngle() {
        return 0;
    }
    public float MaxAngle() {
        return RawAngleOfNormal(trailingLightEdge);
    }
}

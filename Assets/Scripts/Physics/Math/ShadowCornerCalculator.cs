using System.Collections.Generic;

using Unity.Burst;
using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;

using Physics.Math;

using Rect = Physics.Math.Rect;

using Utilities;

public struct ShadowCornerCalculator {

    public struct Outputs {
        public NativeList<TwoWayPenConstraint.Partial>? partialEdgeConstraints;
        public NativeList<ThreeWayPenConstraint.Partial>? partialCornerConstraints;

        [BurstDiscard]
        public List<ShadowEdgeManifold> debugEdgeManifoldCollector {get; set;}
        [BurstDiscard]
        public List<ShadowCornerManifold> debugCornerManifolds {get; set;}
        [BurstDiscard]
        public List<System.ValueTuple<ShadowEdgeManifold, EdgeMount>> debugEdgeMounts {get; set;}
        [BurstDiscard]
        public List<CornerMountTuple> debugCornerMounts {get; set;}

        public struct CornerMountTuple {
            public ShadowCornerManifold m;
            public EdgeMount mount1;
            public EdgeMount mount2;
            public EdgeMount? mount3;
            public ThreeWayPenConstraint.Partial partialConstraint;

            public void Deconstruct(out ShadowCornerManifold m, out EdgeMount mount1, out EdgeMount mount2, out EdgeMount? mount3, out ThreeWayPenConstraint.Partial partialConstraint) {
                m = this.m;
                mount1 = this.mount1;
                mount2 = this.mount2;
                mount3 = this.mount3;
                partialConstraint = this.partialConstraint;
            }
        }

        [BurstDiscard]
        public void DebugCollect(ShadowEdgeManifold m) {
            if (debugEdgeManifoldCollector != null) {
                debugEdgeManifoldCollector.Add(m);
            }
        }
        [BurstDiscard]
        public void DebugCollect(ShadowCornerManifold m) {
            if (debugCornerManifolds != null) {
                debugCornerManifolds.Add(m);
            }
        }

        [BurstDiscard]
        public void DebugCollect(ShadowEdgeManifold manifold, EdgeMount mount) {
            if (debugEdgeMounts != null) {
                debugEdgeMounts.Add(new System.ValueTuple<ShadowEdgeManifold, EdgeMount>(manifold, mount));
            }
        }
        [BurstDiscard]
        public void DebugCollect(ShadowCornerManifold manifold, EdgeMount mount1, EdgeMount mount2, EdgeMount? mount3, ThreeWayPenConstraint.Partial partial) {
            if (debugCornerMounts != null) {
                debugCornerMounts.Add(new CornerMountTuple{m=manifold, mount1=mount1, mount2=mount2, mount3=mount3, partialConstraint=partial});
            }
        }

        public void Collect(in TwoWayPenConstraint.Partial c) {
            if (partialEdgeConstraints != null) {
                partialEdgeConstraints.Value.Add(c);
            }
        }
        public void Collect(in ThreeWayPenConstraint.Partial c) {
            if (partialCornerConstraints != null) {
                partialCornerConstraints.Value.Add(c);
            }
        }
    }

    private struct EdgeCornerIdx {
        public int edgeFirstCornerIdx;
        public int cornerIdx;
    }

    public struct Edge : System.IComparable<Edge> {
        public bool isIlluminationTag;
        public int lightSource;
        public float angle;
        public float2 direction;
        // Indicates the illuminated side of the edge. +1/-1 means illumination
        // for angles larger/smaller than this edge's angle.
        public sbyte lightSide;

        public int CompareTo(Edge other) {
            if (this.lightSource == other.lightSource) {
                return angle.CompareTo(other.angle);
            } else {
                return lightSource.CompareTo(other.lightSource);
            }
        }

        public struct EdgeKey : System.IEquatable<EdgeKey> {
            public int lightSource;
            public float angle;

            public bool Equals(EdgeKey o) {
                return lightSource == o.lightSource && angle == o.angle;
            }

            public override int GetHashCode() {
                return lightSource ^ angle.GetHashCode();
            }
        }

        public EdgeKey GetEdgeKey() {
            return new EdgeKey {
                lightSource = lightSource,
                angle = angle
            };
        }
    }


    // TODO: Might want to make this have an optional second contact
    public struct EdgeMount {
        public Entity castingEntity;
        public EdgeSourceType castingShapeType;
        public float2 shapeCenter;
        public float2 point;
        public int id;
    }

    public struct Corner {
        public float2 point;
        public int prevEdge;
        public int nextEdge;
        public bool isNull;

        public Corner(int prevEdge, int nextEdge, float2 point) {
            this.nextEdge = nextEdge;
            this.prevEdge = prevEdge;
            this.point = point;
            this.isNull = false;
        }

        public bool LiesOnBoxEdge() {
            return nextEdge < 0 || prevEdge < 0;
        }
        public bool IsBoxCorner() {
            return nextEdge < 0 && prevEdge < 0;
        }

        public static Corner Null => new Corner{isNull = true};
    }
    private NativeArray<LightSource> lights;
    private NativeArray<AngleCalculator> lightAngleCalculators;
    private FixedList512<Edge> edges;
    private FixedList512<Corner> islands;
    private Entity boxEntity;
    private NativeMultiHashMap<Edge.EdgeKey, EdgeMount> edgeMounts;

    public FixedList512<Corner> GetIslandsForDebug() {
        return islands;
    }
    private Rect box;

    private Outputs o;

    public ShadowCornerCalculator(Box box, Entity boxEntity, NativeArray<LightSource> lights, NativeArray<AngleCalculator> lightAngleCalculators, MultiHashMapIterator<Entity, Edge> edges, ref NativeMultiHashMap<Edge.EdgeKey, EdgeMount> edgeMounts, in Outputs o) {
        this.box = box.ToRect();
        this.boxEntity = boxEntity;
        this.lights = lights;
        this.edges = new FixedList512<Edge>();
        foreach (var edge in edges) {
            this.edges.Add(edge);
        }

        this.lightAngleCalculators = lightAngleCalculators;
        islands = new FixedList512<Corner>();
        this.edgeMounts = edgeMounts;

        this.o = o;

        Compute();
        ComputeManifolds();
    }

    public float2 GetEdgeDirectionUnnormalized(int edgeIdx) {
        if (edgeIdx < 0) {
            return box.GetEdgeDirection(edgeIdx);
        } else {
            return edges[edgeIdx].direction;
        }
    }
    // Negative edge index indicates it is an edge of the box
    public float2 Intersection(int edge1Idx, int edge2Idx, bool treatAsRays=false) {
        float2 s1;
        float2 r1;
        if (edge1Idx >= 0) {
            Edge edge1 = edges[edge1Idx];
            s1 = lights[edge1.lightSource].pos;
            r1 = edge1.direction;
        } else {
            s1 = box.GetVertex(edge1Idx);
            r1 = box.GetVertex(edge1Idx+1) - s1;

            // If the edges are both box edges and they are not adjacent, then
            // they don't intersect.
            if (edge2Idx < 0 && (edge1Idx - edge2Idx)%2 == 0) {
                return new float2(math.INFINITY, math.INFINITY);
            }
        }

        float2 s2;
        float2 r2;
        if (edge2Idx >= 0) {
            Edge edge2 = edges[edge2Idx];
            s2 = lights[edge2.lightSource].pos;
            r2 = edge2.direction;
        } else {
            s2 = box.GetVertex(edge2Idx);
            r2 = box.GetVertex(edge2Idx+1) - s2;
        }

        float2 d = s2 - s1;
        float det = Lin.Cross(r2, r1);
        
        float u = Lin.Cross(r2, d) / det;
        if (treatAsRays) {
            if (edge1Idx >= 0 && u < 0) {
                return new float2(math.INFINITY, math.INFINITY);
            }

            float v = Lin.Cross(r1, d) / det;
            if (edge2Idx >= 0 && v < 0) {
                return new float2(math.INFINITY, math.INFINITY);
            }
        }

        return new float2(s1 + u*r1);
    }

    private void Compute() {
        var islandsAlternate = new FixedList512<Corner>();

        for (int i = 0; i < 4; i++) {
            // Edge i corresponds to the edge running between box corner i and
            // corner i+1. Thus, the corner that edge i and edge i-1 share is
            // corner i.
            int nextEdge = i;
            int prevEdge = (i - 1 + 4)%4;
            // Subtracting 4 to ensure it is negative, and that it has the same
            // value mod 4.
            islands.Add(new Corner(
                nextEdge: nextEdge - 4,
                prevEdge: prevEdge - 4,
                point: box.GetVertex(i)
            ));
        }
        islands.Add(new Corner{isNull = true});

        edges.Sort();
        for (int edgeIdx = 0; edgeIdx < edges.Length; edgeIdx++) {
            var edge = edges[edgeIdx];
            
            islandsAlternate.Clear();
            ClipIslands(in islands, ref islandsAlternate, edgeIdx);

            bool isLastEdge = edgeIdx >= edges.Length - 1 || edge.lightSource != edges[edgeIdx+1].lightSource;
            if (!isLastEdge) {
                var nextEdge = edges[edgeIdx+1];
                if (nextEdge.lightSide == -1) {
                    // In this case, nextEdge cancels out edge. So we add its
                    // clipped islands as well. This works because of the set
                    // theoretical fact:
                    //     I - (A intersect B) == (I - A) union (I - B)
                    // Where
                    //     I is the region enclosed in an island
                    //     A is the illuminated half plane for 'edge'
                    //     B is the illuminated half plane for 'nextEdge'
                    //
                    // ClipIslands(I, I_out, E) gives the result I_out = I_out union (I - E)
                    ClipIslands(in islands, ref islandsAlternate, edgeIdx+1);
                    edgeIdx++;
                }
            }

            islands = islandsAlternate;
        }
    }

    private void ClipIslands(in FixedList512<Corner> islands, ref FixedList512<Corner> islandsOut, int clipEdgeIdx) {
        for (int i = 0; i < islands.Length; i++) {
            if (i == 0 || islands[i - 1].isNull) {
                ClipIsland(in islands, i, ref islandsOut, clipEdgeIdx);
            }
        }
    }

    private void ClipIsland(in FixedList512<Corner> islands, int islandStart, ref FixedList512<Corner> islandsOut, int clipEdgeIdx) {
        Edge edge = edges[clipEdgeIdx];
        // If the edge is an illumination tag, then the edge illuminates
        // everything, so we don't create any new islands.
        if (edge.isIlluminationTag) {
            return;
        }
        float2 lightPos = lights[edge.lightSource].pos;
        var angleCalculator = lightAngleCalculators[edge.lightSource];

        float2 lightNormal = angleCalculator.NormalTowardsLight(edge.direction, edge.lightSide);

        bool IsClipped(float2 point) {
            return math.dot(point - lightPos, lightNormal) > 0;
        }

        for (int i = islandStart; true; i++) {
            var c1 = islands[i];
            if (c1.isNull) {
                break;
            }
            var c2 = islands[i+1];
            if (c2.isNull) {
                c2 = islands[islandStart];
            }

            bool clip1 = IsClipped(c1.point);
            bool clip2 = IsClipped(c2.point);

            if (!clip1 && clip2) {
                islandsOut.Add(c1);
                int prevEdge = c1.nextEdge;
                int nextEdge = clipEdgeIdx;
                islandsOut.Add(new Corner(
                    prevEdge: prevEdge,
                    nextEdge: nextEdge,
                    point: Intersection(prevEdge, nextEdge)
                ));
            } else if (clip1 && !clip2) {
                int prevEdge = clipEdgeIdx;
                int nextEdge = c1.nextEdge;
                islandsOut.Add(new Corner(
                    prevEdge: prevEdge,
                    nextEdge: nextEdge,
                    point: Intersection(prevEdge, nextEdge)
                ));
            } else if (!clip1 && !clip2) {
                islandsOut.Add(c1);
            }
        }

        if (islandsOut.Length > 0 && !islandsOut.ElementAt(islandsOut.Length - 1).isNull) {
            islandsOut.Add(Corner.Null);
        }
    }

    private void ComputeManifolds() {
        int islandStart = 0;
        for (int i = 0; i < islands.Length; i++) {
            if (islands[i].isNull) {
                ComputeManifoldsForIsland(islandStart, i);
                islandStart = i+1;
            }
        }
    }

    private static int WrapIndex(int index, int start, int end) {
        WrapIndex(ref index, start, end);
        return index;
    }
    private static void WrapIndex(ref int index, int start, int end) {
        int len = end - start;
        index = ((index - start)%len + len)%len + start;
    }

    private EdgeCornerIdx? ComputeBestResolutionForIsland(int islandStart, int islandEnd) {
        EdgeCornerIdx? bestPairIdx = null;
        float minCost = math.INFINITY;

        // The outer loop iterates over all the edges in the island.
        for (int edgeFirstCornerIdx = islandStart; edgeFirstCornerIdx < islandEnd; edgeFirstCornerIdx++) {
            int edgeIdx = islands[edgeFirstCornerIdx].nextEdge;
            float2 edgeCorner1 = islands[edgeFirstCornerIdx].point;

            if (edgeIdx < 0) {
                continue;
            }
            Edge edge = edges[edgeIdx];
            var edgeLightAngleCalc = this.lightAngleCalculators[edge.lightSource];
            float2 edgeLightPos = this.lights[edge.lightSource].pos;

            EdgeCornerIdx? worstPairIdx = null;
            float maxCost = -math.INFINITY;

            float2 edgeNorm = -edgeLightAngleCalc.NormalTowardsLight(edge.direction, edge.lightSide);

            for (int corner2Idx = islandStart; !islands[corner2Idx].isNull; corner2Idx++) {
                Corner corner = islands[corner2Idx];

                //float cost = edgeLightAngleCalc.CostOfRotationTo(edge.direction, corner.point) * -edge.lightSide;

                float cost = math.dot(edgeNorm, corner.point - edgeCorner1);
                if (cost > maxCost) {
                    maxCost = cost;
                    worstPairIdx = new EdgeCornerIdx{edgeFirstCornerIdx = edgeFirstCornerIdx, cornerIdx = corner2Idx};
                }
            }

            // The best pair is the best of the worst pairs
            if (maxCost < minCost) {
                minCost = maxCost;
                bestPairIdx = worstPairIdx;
            }
        }

        return bestPairIdx;
    }


    // TODO: There is a O(n) algorithm we can use here. Currently this is O(n^2).
    private void ComputeManifoldsForIsland(int islandStart, int islandEnd) {

        if (ComputeBestResolutionForIsland(islandStart, islandEnd) is EdgeCornerIdx ecIdx) {

            Corner edgeCorner1 = islands[ecIdx.edgeFirstCornerIdx];
            Corner edgeCorner2 = islands[WrapIndex(ecIdx.edgeFirstCornerIdx+1, islandStart, islandEnd)];

            Edge e = edges[edgeCorner1.nextEdge];
            Corner c = islands[ecIdx.cornerIdx];

            float2 cornerPrevEdgeDir = GetEdgeDirectionUnnormalized(c.prevEdge);
            float2 cornerNextEdgeDir = GetEdgeDirectionUnnormalized(c.nextEdge);

            if (math.abs(Lin.Cross(edgeCorner1.point - edgeCorner2.point, cornerPrevEdgeDir)) < .01f) {
                AddManifold(edgeCorner1.prevEdge, edgeCorner1.nextEdge, c.prevEdge);
                AddManifold(edgeCorner2.prevEdge, edgeCorner2.nextEdge, c.prevEdge);
            } else if (math.abs(Lin.Cross(edgeCorner1.point - edgeCorner2.point, cornerNextEdgeDir)) < .01f) {
                AddManifold(edgeCorner1.prevEdge, edgeCorner1.nextEdge, c.nextEdge);
                AddManifold(edgeCorner2.prevEdge, edgeCorner2.nextEdge, c.nextEdge);
            } else {
                AddManifold(edgeCorner1.nextEdge, c.prevEdge, c.nextEdge);
            }
        }
    }

    private void AddManifold(int3 edges) {
        AddManifold(edges[0], edges[1], edges[2]);
    }
    private void AddManifold(int edge1Idx, int edge2Idx, int edge3Idx) {
        void Swap(ref int a, ref int b) {
            var tmp = a;
            a = b;
            b = tmp;
        }

        // Shifting all box edges to the beginning of the sequence (edge1Idx, edge2Idx, edge3Idx)
        if (edge1Idx >= 0) {
            Swap(ref edge1Idx, ref edge2Idx);
        }
        if (edge2Idx >= 0) {
            Swap(ref edge2Idx, ref edge3Idx);
        }
        if (edge1Idx >= 0) {
            Swap(ref edge1Idx, ref edge2Idx);
        }

        // If some edges are duplicates, don't create a manifold
        if (edge1Idx == edge2Idx || edge2Idx == edge3Idx || edge1Idx == edge3Idx) {
            return;
        }

        if (edge3Idx < 0) {
            // All the edges are box edges in this case, so there is nothing to resolve
            return;
        } else if (edge2Idx < 0) {
            // 2 box edges, one shadow edge

            int boxEdge1 = edge1Idx;
            int boxEdge2 = edge2Idx;
            int shadowEdge = edge3Idx;

            float2 corner = Intersection(boxEdge1, boxEdge2);
            // Sometimes the box edges are opposite each other, and so don't
            // intersect.
            if (!math.isfinite(corner.x) || !math.isfinite(corner.y)) {
                return;
            }
            Edge e = edges[shadowEdge];

            float2 normal = -lightAngleCalculators[e.lightSource].NormalTowardsLight(e.direction, e.lightSide);
            float2 lightSource = lights[e.lightSource].pos;
        
            float delta = -math.dot(corner - lightSource, normal);

            var manifold = new ShadowEdgeManifold {
                p = corner,
                contactIdOn2 = new int3(
                    box.id,
                    boxEdge1,
                    boxEdge2
                ).GetHashCode(),
                x1 = lightSource,
                d1 = e.direction,
                e2 = boxEntity,
                x2 = box.pos,
                n = normal,
                delta = delta,
            };

            o.DebugCollect(manifold);
        
            var prototype = new TwoWayPenConstraint.Partial.Prototype(manifold);

            foreach (EdgeMount mount in It.Iterate(edgeMounts, e.GetEdgeKey())) {
                var p = new TwoWayPenConstraint.Partial(in prototype, in mount, in manifold);
                o.Collect(p);
                o.DebugCollect(manifold, mount);
            }
        } else {
            int lineIdx = edge1Idx;
            int shadowEdge1Idx = edge2Idx;
            int shadowEdge2Idx = edge3Idx;
        
            Edge shadowEdge1 = edges[shadowEdge1Idx];
            Edge shadowEdge2 = edges[shadowEdge2Idx];

            var m = new ShadowCornerManifold {
                d1 = shadowEdge1.direction,
                x1 = lights[shadowEdge1.lightSource].pos,

                d2 = shadowEdge2.direction,
                x2 = lights[shadowEdge2.lightSource].pos,
               
                p1 = Intersection(lineIdx, shadowEdge1Idx),
                p2 = Intersection(lineIdx, shadowEdge2Idx),
                p = Intersection(shadowEdge1Idx, shadowEdge2Idx),
            };
            if (lineIdx < 0) {
                float2 boxEdgeP1 = box.GetVertex(lineIdx);
                float2 boxEdgeP2 = box.GetVertex(lineIdx+1);

                m.contactIdOnBox = new int2(box.id, lineIdx).GetHashCode();
        
                // This works because rect vertices wind counterclockwise
                m.n = Lin.Cross(math.normalize(boxEdgeP2 - boxEdgeP1), 1);
                m.s = boxEdgeP1;

                m.x3 = box.pos;

                var prototype = new ThreeWayPenConstraint.Partial.Prototype(m);

                o.DebugCollect(m);
                // TODO: Check if edge mounts will participate
                foreach (EdgeMount mount1 in It.Iterate(edgeMounts, shadowEdge1.GetEdgeKey())) {
                    foreach (EdgeMount mount2 in It.Iterate(edgeMounts, shadowEdge2.GetEdgeKey())) {
                        var p = new ThreeWayPenConstraint.Partial(in prototype, in mount1, in mount2, boxEntity, in m);
                        o.Collect(p);
                        o.DebugCollect(m, mount1, mount2, null, p);
                    }
                }
            } else {
                var shadowEdge3Idx = lineIdx;
                Edge shadowEdge3 = edges[shadowEdge3Idx];
                m.n = lightAngleCalculators[shadowEdge3.lightSource].NormalTowardsLight(shadowEdge3.direction, shadowEdge3.lightSide);
                m.x3 = lights[shadowEdge3.lightSource].pos;
                m.s = m.x3;

                o.DebugCollect(m);
                var prototype = new ThreeWayPenConstraint.Partial.Prototype(m);
                foreach (EdgeMount mount1 in It.Iterate(edgeMounts, shadowEdge1.GetEdgeKey())) {
                    foreach (EdgeMount mount2 in It.Iterate(edgeMounts, shadowEdge2.GetEdgeKey())) {
                        foreach (EdgeMount mount3 in It.Iterate(edgeMounts, shadowEdge3.GetEdgeKey())) {
                            var p = new ThreeWayPenConstraint.Partial(in prototype, in mount1, in mount2, in mount3, shadowEdge3.direction, in m);
                            o.Collect(p);
                            o.DebugCollect(m, mount1, mount2, mount3, p);
                        }
                    }
                }
            }
        }
    }
}

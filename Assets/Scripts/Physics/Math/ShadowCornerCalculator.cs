using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;

using Physics.Math;
using UnityEngine;

using Rect = Physics.Math.Rect;

using Utilities;

using ShadowEdgeManifold = ShadowEdgeGenerationSystem.ShadowEdgeManifold;
using ShadowCornerManifold = ShadowEdgeGenerationSystem.ShadowCornerManifold;

public struct CornerCalculator {
    private struct EdgeCornerIdx {
        public int edgeFirstCornerIdx;
        public int cornerIdx;
    }

    public struct Edge : System.IComparable<Edge> {
        public enum Type : byte {
            illuminationTag,
            edge
        }
        public Type type;
        public int lightSource;
        public float angle;
        public float2 direction;
        // Indicates the illuminated side of the edge. +1/-1 means illumination
        // for angles larger/smaller than this edge's angle.
        public sbyte lightSide;

        // Required for converting to a ShadowEdgeManifold
        public Entity castingEntity;
        public float2 mount1;
        public float2? mount2;
        public ShapeType castingShapeType;
        public int id;

        public int CompareTo(Edge other) {
            if (this.lightSource == other.lightSource) {
                return angle.CompareTo(other.angle);
            } else {
                return lightSource.CompareTo(other.lightSource);
            }
        }
    }

    public struct Corner {
        public float2 point;
        public int nextEdge;
        public int prevEdge;
        public bool isNull;

        public Corner(int nextEdge, int prevEdge, float2 point) {
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

    private struct Wedge {
        public float angleNegative;
        public float anglePositive;
        public int negativeEdge;
        public int positiveEdge;
        public int lightSource;
        public int SideOf(float angle) {
            Debug.Assert(angleNegative < anglePositive);
            if (angle < angleNegative) {
                return -1;
            } else if (angle > anglePositive) {
                return 1;
            } else {
                return 0;
            }
        }
    }
    private NativeArray<LightSource> lights;
    private NativeArray<AngleCalculator> lightAngleCalculators;
    private FixedList512<Edge> edges;
    private FixedList512<Corner> islands;
    private Entity boxEntity;
    public FixedList512<Corner> GetIslandsForDebug() {
        return islands;
    }
    private Rect box;

    public CornerCalculator(Box box, Entity boxEntity, NativeArray<LightSource> lights, NativeArray<AngleCalculator> lightAngleCalculators, MultiHashMapIterator<Entity, Edge> edges) {
        this.box = box.ToRect();
        this.boxEntity = boxEntity;
        this.lights = lights;
        this.edges = new FixedList512<Edge>();
        foreach (var edge in edges) {
            this.edges.Add(edge);
        }

        this.lightAngleCalculators = lightAngleCalculators;
        islands = new FixedList512<Corner>();

        Compute();
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

            Wedge wedge;
            if (edge.lightSide == -1) {
                bool firstEdge = edgeIdx == 0 || edge.lightSource != edges[edgeIdx-1].lightSource;
                // This should not happen very often. This might happen if
                // shapes are deeply overlapping.
                if (firstEdge) {
                    wedge = new Wedge{
                        angleNegative = -math.INFINITY,
                        anglePositive = edge.angle,
                        negativeEdge = 1337,
                        positiveEdge = edgeIdx,
                        lightSource = edge.lightSource
                    };
                } else {
                    continue;
                }
            } else {
                bool lastEdge = edgeIdx >= edges.Length || edge.lightSource != edges[edgeIdx].lightSource;
                // This also should not happen very often. For the same reasons
                // as above.
                if (lastEdge) {
                    wedge = new Wedge{
                        angleNegative = edge.angle,
                        anglePositive = math.INFINITY,
                        negativeEdge = edgeIdx,
                        positiveEdge = 1337,
                        lightSource = edge.lightSource
                    };
                } else {
                    var nextEdge = edges[edgeIdx+1];
                    wedge = new Wedge{
                        angleNegative = edge.angle,
                        anglePositive = nextEdge.angle,
                        negativeEdge = edgeIdx,
                        positiveEdge = edgeIdx+1,
                        lightSource = edge.lightSource
                    };
                }
            }
            
            SplitIslands(ref islands, ref islandsAlternate, wedge);
            islands = islandsAlternate;
        }
    }

    private void SplitIslands(ref FixedList512<Corner> islands, ref FixedList512<Corner> islandsOut, Wedge wedge) {
        islandsOut.Clear();
        for (int i = 0; i < islands.Length; i++) {
            if (i == 0 || islands[i - 1].isNull) {
                SplitIsland(ref islands, i, ref islandsOut, wedge);
            }
        }
    }

    private void SplitIsland(ref FixedList512<Corner> islands, int islandStart, ref FixedList512<Corner> islandsOut, Wedge wedge) {
        var islandNegOut = new FixedList512<Corner>();
        var islandPosOut = new FixedList512<Corner>();

        var angleCalculator = lightAngleCalculators[wedge.lightSource];

        for (int i = islandStart; true; i++) {
            var corner = islands[i];
            if (corner.isNull) {
                break;
            }
            var nextCorner = islands[i+1];
            if (nextCorner.isNull) {
                nextCorner = islands[islandStart];
            }

            var angle = angleCalculator.Angle(corner.point);
            var nextAngle = angleCalculator.Angle(nextCorner.point);

            int side = wedge.SideOf(angle);
            int nextSide = wedge.SideOf(nextAngle);

            if (side == -1) {
                islandNegOut.Add(corner);
            } else if (side == 1) {
                islandPosOut.Add(corner);
            }

            if (nextSide != side) {
                // If the edge connecting corner and nextCorner crosses the
                // wedge's negative edge, create a corner at that intersection.
                if (side == -1 || nextSide == -1) {
                    int nextEdge;
                    int prevEdge;
                    if (nextSide == -1) {// Going to the negative side
                        nextEdge = corner.nextEdge;
                        prevEdge = wedge.negativeEdge;
                    } else {// Coming from the negative side
                        nextEdge = wedge.negativeEdge;
                        prevEdge = corner.nextEdge;
                    }
                    islandNegOut.Add(new Corner(
                        nextEdge: nextEdge,
                        prevEdge: prevEdge,
                        point: Intersection(nextEdge, prevEdge)
                    ));
                } 
                if (side == 1 || nextSide == 1) {
                    int nextEdge;
                    int prevEdge;
                    if (nextSide == 1) { // Going to the positive side
                        nextEdge = corner.nextEdge;
                        prevEdge = wedge.positiveEdge;
                    } else { // Coming from the positive side
                        nextEdge = wedge.positiveEdge;
                        prevEdge = corner.nextEdge;
                    }
                    islandPosOut.Add(new Corner(
                        nextEdge: nextEdge,
                        prevEdge: prevEdge,
                        point: Intersection(nextEdge, prevEdge)
                    ));
                }
            }
        }

        if (islandNegOut.Length > 0) {
            foreach (var item in islandNegOut) {
                islandsOut.Add(item);
            }
            islandsOut.Add(Corner.Null);
        }

        if (islandPosOut.Length > 0) {
            foreach (var item in islandPosOut) {
                islandsOut.Add(item);
            }
            islandsOut.Add(Corner.Null);
        }
    }

    public void ComputeManifolds(
            ref NativeList<ShadowEdgeManifold> edgeManifolds,
            ref NativeList<ShadowCornerManifold> cornerManifolds) {

        int islandStart = 0;
        for (int i = 0; i < islands.Length; i++) {
            if (islands[i].isNull) {
                ComputeManifoldsForIsland(islandStart, i, ref edgeManifolds, ref cornerManifolds);
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
            if (edgeIdx < 0) {
                continue;
            }
            Edge edge = edges[edgeIdx];
            var edgeLightAngleCalc = this.lightAngleCalculators[edge.lightSource];
            float2 edgeLightPos = this.lights[edge.lightSource].pos;

            EdgeCornerIdx? worstPairIdx = null;
            float maxCost = -math.INFINITY;

            for (int corner2Idx = islandStart; !islands[corner2Idx].isNull; corner2Idx++) {
                Corner corner = islands[corner2Idx];

                float cost = edgeLightAngleCalc.CostOfRotationTo(edge.direction, corner.point) * -edge.lightSide;

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
    private void ComputeManifoldsForIsland(int islandStart, int islandEnd,
            ref NativeList<ShadowEdgeManifold> edgeManifolds,
            ref NativeList<ShadowCornerManifold> cornerManifolds) {

        if (ComputeBestResolutionForIsland(islandStart, islandEnd) is EdgeCornerIdx ecIdx) {

            Corner edgeCorner1 = islands[ecIdx.edgeFirstCornerIdx];
            Corner edgeCorner2 = islands[WrapIndex(ecIdx.edgeFirstCornerIdx+1, islandStart, islandEnd)];

            Edge e = edges[edgeCorner1.nextEdge];
            Corner c = islands[ecIdx.cornerIdx];

            AddManifold(edgeCorner1.nextEdge, c.prevEdge, c.nextEdge, ref edgeManifolds, ref cornerManifolds);

            if (c.nextEdge < 0 && c.prevEdge < 0) {
                if (edgeCorner2.nextEdge != c.prevEdge) {
                    AddManifold(c.prevEdge, edgeCorner2.nextEdge, edgeCorner2.prevEdge, ref edgeManifolds, ref cornerManifolds);
                }
                if (edgeCorner1.prevEdge != c.nextEdge) {
                    AddManifold(c.nextEdge, edgeCorner1.nextEdge, edgeCorner1.prevEdge, ref edgeManifolds, ref cornerManifolds);
                }
            }
        }
    }

    private void AddManifold(int edge1Idx, int edge2Idx, int edge3Idx, ref NativeList<ShadowEdgeManifold> edgeManifolds, ref NativeList<ShadowCornerManifold> cornerManifolds) {
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

            float2 normal = Lin.Cross(e.direction, e.lightSide);
            float2 lightSource = lights[e.lightSource].pos;
        
            float overlap = math.dot(corner - lightSource, normal);
        
            edgeManifolds.Add(new ShadowEdgeManifold {
                castingEntity = e.castingEntity,
                castingShapeType = e.castingShapeType,
                contact1 = new Geometry.Contact{
                    point = corner,
                    id = new int4(
                        e.id,
                        box.id,
                        boxEdge1,
                        boxEdge2
                    ).GetHashCode() ^ 30345 //Arbitrary number. Don't use anywhere else.
                },
                contact2 = null,
                lightSource = lightSource,
                mount1 = e.mount1,
                mount2 = e.mount2,
                shadHitEntity = boxEntity,
                normal = normal,
                overlap = overlap
            });
        } else {
            // 1 box or shadow edge, 2 shadow edges.
            // This is handling 2 cases at once, since the cases are very similar
            int lineIdx = edge1Idx;
            int shadowEdge1Idx = edge2Idx;
            int shadowEdge2Idx = edge3Idx;

            Edge shadowEdge1 = edges[shadowEdge1Idx];
            Edge shadowEdge2 = edges[shadowEdge2Idx];


            var m = new ShadowCornerManifold {
                castingEntity1 = shadowEdge1.castingEntity,
                castingEntity1Type = shadowEdge1.castingShapeType,
                casting1Corner = Intersection(lineIdx, shadowEdge1Idx, treatAsRays:true),
                e1Mount1 = shadowEdge1.mount1,
                e1Mount2 = shadowEdge1.mount2,

                castingEntity2 = shadowEdge2.castingEntity,
                castingEntity2Type = shadowEdge2.castingShapeType,
                casting2Corner = Intersection(lineIdx, shadowEdge2Idx, treatAsRays:true),
                e2Mount1 = shadowEdge2.mount1,
                e2Mount2 = shadowEdge2.mount2,

                lineOppositeCorner = Intersection(shadowEdge1Idx, shadowEdge2Idx, treatAsRays:true),
            };

            if (lineIdx < 0) {
                float2 boxEdgeP1 = box.GetVertex(lineIdx);
                float2 boxEdgeP2 = box.GetVertex(lineIdx+1);

                m.lineEntity = boxEntity;
                m.lineIsShadowEdge = false;
                // Purposely not setting lineEntityType because it is not used
                // when the line is not a shadow edge.
                //m.lineEntityType = null
                // This works because rect vertices wind counterclockwise
                m.normal = Lin.Cross(math.normalize(boxEdgeP2 - boxEdgeP1), 1);
                m.linePoint = boxEdgeP1;
                m.id = new int3(box.id, shadowEdge1.id, shadowEdge2.id).GetHashCode()^40235;
            } else {
                Edge shadowEdge3 = edges[lineIdx];
                m.lineEntity = shadowEdge3.castingEntity;
                m.lineIsShadowEdge = true;
                m.lineEntityCastingType = shadowEdge3.castingShapeType;
                m.normal = lightAngleCalculators[shadowEdge3.lightSource].NormalTowardsLight(shadowEdge3.direction, shadowEdge3.lightSide);
                m.linePoint = shadowEdge3.mount1;
                m.id = new int3(shadowEdge1.id, shadowEdge2.id, shadowEdge3.id).GetHashCode()^506744;
            }

            cornerManifolds.Add(m);
        }
    }
}

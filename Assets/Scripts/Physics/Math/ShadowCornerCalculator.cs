using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;

using Physics.Math;
using UnityEngine;

using Rect = Physics.Math.Rect;

using Utilities;

using ShadowEdgeManifold = ShadowEdgeGenerationSystem.ShadowEdgeManifold;

using EdgeCorner = System.ValueTuple<CornerCalculator.Edge, CornerCalculator.Corner>;

public struct CornerCalculator {
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
    public float2 Intersection(int edge1Idx, int edge2Idx) {
        float2 s1;
        float2 r1;
        if (edge1Idx >= 0) {
            Edge edge1 = edges[edge1Idx];
            s1 = lights[edge1.lightSource].pos;
            r1 = edge1.direction;
        } else {
            s1 = box.GetVertex(edge1Idx);
            r1 = box.GetVertex(edge1Idx+1) - s1;
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
        //float v = Lin.Cross(edge1.direction, d) / det;

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
            // Subtracting 8 to ensure it is negative, and that it has the same
            // value mod 4.
            islands.Add(new Corner(
                nextEdge: nextEdge - 8,
                prevEdge: prevEdge - 8,
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
            ref NativeList<ShadowEdgeManifold> edgeManifolds
            //NativeList<ShadowCornerManifold> boxCornerManifolds,
            ) {

        for (int i = 0; i < islands.Length; i++) {
            if (i == 0 || islands[i-1].isNull) {
                ComputeManifoldsForIsland(i, ref edgeManifolds);
            }
        }
    }

    // TODO: There is a O(n) algorithm we can use here. Currently this is O(n^2).
    private void ComputeManifoldsForIsland(int islandStart,
            ref NativeList<ShadowEdgeManifold> edgeManifolds) {

        EdgeCorner? bestPair = null;
        int2? bestPairIdx = null;
        float minCost = math.INFINITY;


        for (int corner1Idx = islandStart; !islands[corner1Idx].isNull; corner1Idx++) {
            int edgeIdx = islands[corner1Idx].nextEdge;
            if (edgeIdx < 0) {
                continue;
            }
            Edge edge = edges[edgeIdx];
            var edgeLightAngleCalc = this.lightAngleCalculators[edge.lightSource];
            float2 edgeLightPos = this.lights[edge.lightSource].pos;

            EdgeCorner? worstPair = null;
            int2? worstPairIdx = null;
            float maxCost = -math.INFINITY;

            for (int corner2Idx = islandStart; !islands[corner2Idx].isNull; corner2Idx++) {
                Corner corner = islands[corner2Idx];

                float cost = edgeLightAngleCalc.CostOfRotationTo(edge.direction, corner.point) * -edge.lightSide;

                if (cost > maxCost) {
                    maxCost = cost;
                    worstPair = new EdgeCorner(edge, corner);
                    worstPairIdx = new int2(corner1Idx, corner2Idx);
                }
            }

            // The best pair is the best of the worst pairs
            if (maxCost < minCost) {
                minCost = maxCost;
                bestPair = worstPair;
                bestPairIdx = worstPairIdx;
            }
        }

        if (bestPair is EdgeCorner ec) {
            Edge e = ec.Item1;
            Corner c = ec.Item2;

            Corner edgeCorner1 = islands[bestPairIdx.Value.x];
            Corner edgeCorner2 = islands[bestPairIdx.Value.x+1];
            if (edgeCorner2.isNull) {
                edgeCorner2 = islands[islandStart];
            }

            int edgePrevEdge = edgeCorner1.prevEdge;
            int edgeNextEdge = edgeCorner2.nextEdge;

            Geometry.Contact? contact1 = null;
            Geometry.Contact? contact2 = null;

            if (edgePrevEdge < 0) {
                contact1 = new Geometry.Contact{
                    id = new int3(
                        e.id,
                        box.id,
                        edgePrevEdge
                    ).GetHashCode() ^ 69489, //Arbitrary number. Don't use anywhere else.
                    point = edgeCorner1.point
                };
            }

            if (edgeNextEdge < 0) {
                if (contact1 != null) {
                    contact2 = contact1;
                }
                contact1 = new Geometry.Contact{
                    id = new int3(
                        e.id,
                        box.id,
                        edgeNextEdge
                    ).GetHashCode() ^ 47446, //Arbitrary number. Don't use anywhere else.
                    point = edgeCorner2.point
                };
            }


            // If the edge and the corner simply form a triangle, use a single corner contact
            if (c.nextEdge < 0 && c.prevEdge < 0 && edgePrevEdge == c.nextEdge && edgeNextEdge == c.prevEdge) {
                contact1 = new Geometry.Contact{
                    id = new int4(
                        e.id,
                        box.id,
                        edgePrevEdge,
                        edgeNextEdge
                    ).GetHashCode() ^ 4391, //Arbitrary number. Don't use anywhere else.
                    point = c.point
                };
                contact2 = null;
            }

            if (contact1 != null) {
                // Normal points in the direction the shadow edge should resolve in.
                float2 normal = Lin.Cross(e.direction, e.lightSide);
                float2 lightSource = lights[e.lightSource].pos;

                float overlap = math.dot(c.point - lightSource, normal);

                edgeManifolds.Add(new ShadowEdgeManifold {
                    castingEntity = e.castingEntity,
                    castingShapeType = e.castingShapeType,
                    contact1 = contact1.Value,
                    contact2 = contact2,
                    lightSource = lightSource,
                    mount1 = e.mount1,
                    mount2 = e.mount2,
                    shadHitEntity = boxEntity,
                    normal = normal,
                    overlap = overlap
                });
            }
        }
    }
}

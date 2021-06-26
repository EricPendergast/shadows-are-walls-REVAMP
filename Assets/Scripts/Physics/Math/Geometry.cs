using Unity.Mathematics;
using Unity.Collections;
using UnityEngine;

namespace Physics.Math {

    public readonly struct Rect {
        public readonly float2 pos;
        public readonly float2 width;
        public readonly float2 height;
        public readonly int id;

        // Counterclockwise winding. Things depend on this exact ordering.
        public float2 c1 => pos + width + height;
        public float2 c2 => pos - width + height;
        public float2 c3 => pos - width - height;
        public float2 c4 => pos + width - height;

        public Rect(float2 p, float2 w, float2 h, int id) {
            pos = p;
            // Ensuring counterclockwise winding
            if (Math.Lin.Cross(w, h) > 0) {
                width = w;
                height = h;
            } else {
                width = h;
                height = w;
            }
            this.id = id;
        }

        public static Rect FromWidthHeightAngle(float2 p, float width, float height, float angle, int id) {
            float2 right = math.mul(float2x2.Rotate(angle), new float2(1, 0))*width/2;
            float2 up = math.mul(float2x2.Rotate(angle), new float2(0, 1))*height/2;
            return new Rect(p, right, up, id);
        }

        public static Rect FromLineSegment(float2 p1, float2 p2, int id) {
            float2 right = (p2 - p1)/2;
            float2 up = Lin.Cross(math.normalize(right), -1)*.001f;
            return new Rect(p1 + right, right, up, id);
        }


        // Returns the edge with counter clockwise winding
        public LineSegment FurthestEdge(float2 normal, out int edgeIdx) {
            int furthestIdx = FurthestVertex(normal);
            float2 furthest = GetVertex(furthestIdx);

            float2 vLeft = GetVertex(furthestIdx + 1);
            float2 vRight = GetVertex(furthestIdx - 1);

            float2 l = math.normalize(furthest - vLeft);
            float2 r = math.normalize(furthest - vRight);

            if (math.dot(r, normal) <= math.dot(l, normal)) {
                edgeIdx = (furthestIdx - 1 + 4)%4;
                return new LineSegment(vRight, furthest);
            } else {
                edgeIdx = furthestIdx;
                return new LineSegment(furthest, vLeft);
            }
        }

        public float2 FurthestVertexPoint(float2 normal) {
            return pos + math.sign(math.dot(width, normal))*width + math.sign(math.dot(height, normal))*height;
        }

        public int FurthestVertex(float2 normal) {
            if (math.dot(width, normal) > 0) {
                return math.dot(height, normal) > 0 ? 0 : 3;
            } else {
                return math.dot(height, normal) > 0 ? 1 : 2;
            }
        }

        public float2 GetEdgeDirection(int index) {

            switch(index & 3 /* equivalent to index mod 4 (not remainder) */) {
                case 0: return -width;
                case 1: return -height;
                case 2: return width;
                case 3: return height;
                default:
                    Debug.LogError("This should never happen");
                    return math.NAN;
            }
        }

        // ccwVec and cwVec are the directions (not normalized) of the surface
        // counterclockwise from and clockwise from the closest point on the
        // rectangle. For instance, if the closest point is a corner, these
        // vectors will be perpendicular. Otherwise they will be parallel and
        // opposite.
        public float2 ClosestPoint(float2 point, out float2 ccwVec, out float2 cwVec) {
            point -= this.pos;

            float widthMag = math.length(this.width);
            float2 widthNorm = this.width/widthMag;
            float2 width = this.width;
            float wDist = math.dot(widthNorm, point);
            if (wDist < 0) {
                widthNorm *= -1;
                width *= -1;
                wDist *= -1;
            }

            float heightMag = math.length(this.height);
            float2 heightNorm = this.height/heightMag;
            float2 height = this.height;
            float hDist = math.dot(heightNorm, point);
            if (hDist < 0) {
                heightNorm *= -1;
                height *= -1;
                hDist *= -1;
            }

            bool withinWidth = wDist < widthMag;
            bool withinHeight = hDist < heightMag;

            if (withinWidth && withinHeight) {
                if (widthMag - wDist < heightMag - hDist) {
                    ccwVec = Lin.Cross(width, -1);
                    cwVec = -ccwVec;
                    return this.pos + width + hDist*heightNorm;
                } else {
                    ccwVec = Lin.Cross(height, -1);
                    cwVec = -ccwVec;
                    return this.pos + height + wDist*widthNorm;
                }
            } else if (withinWidth) {
                ccwVec = Lin.Cross(height, -1);
                cwVec = -ccwVec;
                return this.pos + height + wDist*widthNorm;
            } else if (withinHeight) {
                ccwVec = Lin.Cross(width, -1);
                cwVec = -ccwVec;
                return this.pos + width + hDist*heightNorm;
            } else {
                if (Lin.Cross(width, height) > 0) {
                    ccwVec = -width;
                    cwVec = -height;
                } else {
                    ccwVec = -height;
                    cwVec = -width;
                }
                return this.pos + width + height;
            }
        }

        public float2 GetVertex(int index) {
            index = index & 3;
        
            int heightSign = 1 - ((index >> 1) << 1);
            int widthSign = (1 - ((index & 1) << 1))*heightSign;
        
            return pos + width*widthSign + height*heightSign;

            // This uses fancy bitwise stuff. Original version of this function:
            //switch (((index%4)+4)%4) {
            //    case 0: return c1;
            //    case 1: return c2;
            //    case 2: return c3;
            //    case 3: return c4;
            //    default: 
            //        Debug.LogError("This should never happen");
            //        return float2.zero;
            //}
        }

        public bool Contains(float2 point) {
            return  math.abs(math.dot(point - pos, width)) < math.lengthsq(width) &&
                    math.abs(math.dot(point - pos, height)) < math.lengthsq(height);
        }

        // Returns the shortest vector that 'point' must be translated by so
        // that it is on the perimeter of this rectangle.
        //public float2 SeparationVector(float2 point) {
        //    point = point - pos;
        //    float xDir = math.dot(point, width) < 0 ? -1 : 1;
        //    float yDir = math.dot(point, height) < 0 ? -1 : 1;
        //
        //    float2 norm1 = xDir*width - math.project(point, xDir*width);
        //    float2 norm2 = yDir*height - math.project(point, yDir*height);
        //
        //    if (math.lengthsq(norm1) < math.lengthsq(norm2)) {
        //        return norm1;
        //    } else {
        //        return norm2;
        //    }
        //}
    }

    public readonly struct LineSegment {
        public readonly float2 p1;
        public readonly float2 p2;

        public LineSegment(float2 p1, float2 p2) {
            this.p1 = p1;
            this.p2 = p2;
        }

        public float2 Furthest(float2 normal) {
            if (math.dot(normal, p1) > math.dot(normal, p2)) {
                return p1;
            } else {
                return p2;
            }
        }
    }

    public struct Projection {
        private float _x1;
        private float _x2;

        public float x1 => _x1;
        public float x2 => _x2;

        public Projection(float x1, float x2) {
            _x1 = math.min(x1, x2);
            _x2 = math.max(x1, x2);
        }

        public bool Overlaps(Projection other) {
            if (x2 < other.x1 || x1 > other.x2) {
                return false;
            } else {
                return true;
            }
        }

        // Returns the smallest offset that 'other' must move by so it touches
        // this projection at a single point. If the objects are overlapping,
        // the sign of the returned value indicates the relative positions of
        // the two projections.
        public float GetSeparationVector(Projection other) {
            float leftMove = x1 - other.x2;
            float rightMove = x2 - other.x1;
            if (math.abs(leftMove) < math.abs(rightMove)) {
                return leftMove;
            } else {
                return rightMove;
            }
        }

        public float GetOverlap(Projection other) {
            // These are the two ways to move 'other' so that it touches 'this'
            // at one point.
            float leftMove = other.x2 - x1;
            float rightMove = x2 - other.x1;
            if (math.abs(leftMove) < math.abs(rightMove)) {
                return leftMove;
            } else {
                return rightMove;
            }
        }

        private bool Contains(float x) {
            return x >= x1 && x >= x2;
        }
    }
    public static partial class Geometry {

        public struct Contact {
            public float2 point;
            public int id;
        }
        public struct Manifold {
            public float2 normal;
            // This is negative if they aren't touching
            public float overlap;
            public Contact contact1;
            public Contact? contact2;
        }

        public static float GetOverlapOnAxis(in Rect r1, in Rect r2, float2 normal) {
            return Project(r1, normal).GetOverlap(Project(r2, normal));
        }

        public static Manifold? GetIntersectData(in Rect r1, in Rect r2, float skin=.0f) {
        
            FixedList64<float2> axes = new FixedList32<float2>();
            axes.Add(r1.width);
            axes.Add(r1.height);
            axes.Add(r2.width);
            axes.Add(r2.height);
            for (int i = 0; i < 4; i++) {
                axes[i] = math.normalize(axes[i]);
            }
        
            // Says how far r1 must move along the best axis before it touches
            // r2 at a single point.
            float minOverlapWeighted = math.INFINITY;
            float minOverlap= math.INFINITY;
            float2 bestAxis = float2.zero;
            
            for (int i = 0; i < axes.Length; i++) {
                float2 ax = axes[i];
                var r1Proj = Project(r1, ax);
                var r2Proj = Project(r2, ax);

                // Shortest translation of r2Proj so it touches r1Proj at one point
                float separation = r1Proj.GetSeparationVector(r2Proj);
                float overlap = separation;

                if (r1Proj.Overlaps(r2Proj)) {
                    // Overlap should be positive if they overlap
                    if (overlap < 0) {
                        overlap = -overlap;
                        ax = -ax;
                    }
                } else {
                    // Overlap should be negative if they don't overlap
                    if (overlap > 0) {
                        overlap = -overlap;
                        ax = -ax;
                    }
                    if (overlap < -skin) {
                        return null;
                    }
                }

                // Weighting the overlap so that there is a slight preference
                // for a downward pointing normal vector.
                var overlapWeighted = overlap + .05f * math.dot(new float2(0, 1), ax);

                if (overlapWeighted < minOverlapWeighted) {
                    minOverlapWeighted = overlapWeighted;
                    minOverlap = overlap;
                    bestAxis = ax;
                }
            }

            ComputeContacts(r1, r2, bestAxis, out var contact1, out var contact2, skin);

            if (contact1 == null) {
                return null;
            } else {
                Manifold manifold = new Manifold{normal = bestAxis, contact1 = (Contact)contact1, contact2 = contact2, overlap=minOverlap};
                return manifold;
            }
        }

        public readonly struct ContactId : System.IEquatable<ContactId> {
            // We need the shape ids because sometimes both edges are on the
            // same shape. This occasionally leads to duplicate contact ids.
            readonly public int shape1Id;
            readonly public int shape2Id;
            readonly public EdgeId edge1;
            readonly public EdgeId edge2;

            public ContactId(int shape1Id, int shape2Id, EdgeId e1, EdgeId e2) {
                if (e1.shapeId < e2.shapeId || 
                        (e1.shapeId == e2.shapeId && e1.edgeIndex < e2.edgeIndex)) {
                    edge1 = e1;
                    edge2 = e2;
                } else {
                    edge1 = e2;
                    edge2 = e1;
                }

                this.shape1Id = math.min(shape1Id, shape2Id);
                this.shape2Id = math.max(shape1Id, shape2Id);
            }

            public override int GetHashCode() {
                return new int2x3(shape1Id, shape2Id, edge1.shapeId, edge1.edgeIndex, edge2.shapeId, edge2.edgeIndex).GetHashCode();
            }

            public bool Equals(ContactId other) {
                return edge1.Equals(other.edge1) && edge2.Equals(other.edge2);
            }

            public override string ToString() {
                return "{edge1: " + edge1.ToString() + ", edge2: " + edge2.ToString() + "}";
            }
        }

        public readonly struct EdgeId : System.IEquatable<EdgeId> {
            readonly public int shapeId;
            readonly public int edgeIndex;
            public EdgeId(int si, int ei) {
                shapeId = si;
                edgeIndex = ei;
            }

            public EdgeId WithIndexOffset(int offset) {
                return new EdgeId(shapeId, ((edgeIndex + offset)%4 + 4)%4);
            }

            public bool Equals(EdgeId other) {
                return shapeId == other.shapeId && edgeIndex == other.edgeIndex;
            }

            public override string ToString() {
                return "{shapeId: " + shapeId + ", edgeIndex: " + edgeIndex + "}";
            }
        }

        //private static ContactId GetContactId(int shape1Id, int shape2Id, EdgeId e1, EdgeId e2) {
        //    return new ContactId(shape1Id, shape2Id, e1, e2);
        //}
        // Debug/optimization note: If you want to make contact points be ints,
        // use this version of GetContactId instead of the above function:
        private static int GetContactId(int shape1Id, int shape2Id, EdgeId e1, EdgeId e2) {
            return new ContactId(shape1Id, shape2Id, e1, e2).GetHashCode();
        }

        private static void ComputeContacts(in Rect r1, in Rect r2, float2 normal, out Contact? contact1, out Contact? contact2, float skin) {
            contact1 = null;
            contact2 = null;

            ComputeReferenceAndIncident(in r1, in r2, ref normal, out LineSegment reference, out LineSegment incident, out EdgeId referenceId, out EdgeId incidentId);

            LineSegment incidentPrev = incident;

            float2 refEdgeVec = math.normalize(reference.p1 - reference.p2);

            
            if (!Clip(ref incident, refEdgeVec, math.dot(refEdgeVec, reference.p2), out bool refP2IncP1Clip, out bool refP2IncP2Clip)) {
                return;
            }

            if (!Clip(ref incident, -refEdgeVec, math.dot(-refEdgeVec, reference.p1), out bool refP1IncP1Clip, out bool refP1IncP2Clip)) {
                return;
            }

            float2 refNorm = Lin.Cross(refEdgeVec, -1);
            double max = math.dot(refNorm, reference.p1) + skin;

            if (math.dot(refNorm, incident.p1) < max) {

                EdgeId contactEdgeId;
                if (refP1IncP1Clip) {
                    contactEdgeId = referenceId.WithIndexOffset(-1);
                } else if (refP2IncP1Clip) {
                    contactEdgeId = referenceId.WithIndexOffset(1);
                } else {
                    contactEdgeId = incidentId.WithIndexOffset(-1);
                }

                contact1 = new Contact{point=incident.p1, id=GetContactId(r1.id, r2.id, incidentId, contactEdgeId)};
            }

            if (math.dot(refNorm, incident.p2) < max) {

                EdgeId contactEdgeId;
                if (refP1IncP2Clip) {
                    contactEdgeId = referenceId.WithIndexOffset(-1);
                } else if (refP2IncP2Clip) {
                    contactEdgeId = referenceId.WithIndexOffset(1);
                } else {
                    contactEdgeId = incidentId.WithIndexOffset(1);
                }

                contact2 = new Contact{point=incident.p2, id=GetContactId(r1.id, r2.id, incidentId, contactEdgeId)};
            }

            if (contact1 == null) {
                contact1 = contact2;
                contact2 = null;
            }
        }

        // Line segments are returned with counterclockwise winding, relative
        // to the rect they are part of
        private static void ComputeReferenceAndIncident(in Rect r1, in Rect r2, ref float2 normal, out LineSegment reference, out LineSegment incident, out EdgeId referenceId, out EdgeId incidentId) {
            LineSegment e1 = r1.FurthestEdge(normal, out int e1Idx);
            LineSegment e2 = r2.FurthestEdge(-normal, out int e2Idx);

            float2 e1Vec = math.normalize(e1.p2 - e1.p1);
            float2 e2Vec = math.normalize(e2.p2 - e2.p1);

            reference = e1;
            incident = e2;

            referenceId = new EdgeId(r1.id, e1Idx);
            incidentId = new EdgeId(r2.id, e2Idx);
        }

        // Clips 'seg' so that no point on 'seg' lies further along 'normal' than 'maxDist'
        // Returns false if the entire line segment was clipped.
        // Keeps the orientation of the line segment (e.g. if seg.p1 is clipped
        // to p1', the returned line segment will be (p1', seg.p2) and never
        // (seg.p2, p1') ).
        private static bool Clip(ref LineSegment seg, float2 normal, float maxDist, out bool clippedP1, out bool clippedP2) {
            float d1 = math.dot(normal, seg.p1) - maxDist;
            float d2 = math.dot(normal, seg.p2) - maxDist;

            clippedP1 = d1 < 0;
            clippedP2 = d2 < 0;

            if (d1 * d2 < 0) {
                float2 p1 = d1 >= 0 ? seg.p1 : seg.p2;

                float2 p2 = seg.p2 - seg.p1;
                float u = d1 / (d1 - d2);
                p2 *= u;
                p2 += seg.p1;
                seg = d1 >= 0 ? new LineSegment(p1, p2) : new LineSegment(p2, p1);
                return true;
            } else {
                if (d1 < 0) {
                    seg = new LineSegment(float2.zero, float2.zero);
                    return false;
                } else {
                    return true;
                }
            }
        }

        private static Projection Project(in Rect r, float2 normal) {
            float p1 = Project(r.c1, normal);
            float p2 = Project(r.c2, normal);
            float p3 = Project(r.c3, normal);
            float p4 = Project(r.c4, normal);

            return new Projection(
                math.min(p1, math.min(p2, math.min(p3, p4))),
                math.max(p1, math.max(p2, math.max(p3, p4)))
            );
        }

        private static float Project(float2 point, float2 normal) {
            return math.dot(point, normal);
        }
    }
}


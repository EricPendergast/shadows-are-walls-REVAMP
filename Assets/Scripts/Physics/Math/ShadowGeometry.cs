using Unity.Mathematics;
using Unity.Collections;
using UnityEngine;

namespace Physics.Math {

    public static partial class Geometry {
        public struct ShadowGeometry {
            public float2 contact1;
            public float2? contact2;
            public int id1;
            public int? id2;
        }

        // The slop term is so that if two edges of the rect are close to the shadow edge, then both edges are stored in ShadowGeometry.
        public static void CalculateShadowGeometry(Rect rect, float2 lightSource, float slop, out ShadowGeometry sg1, out ShadowGeometry sg2) {
            if (rect.Contains(lightSource)) {
                Debug.Log("TODO: Opaque object contains light source. Behavior will not be correct.");
            }

            FixedList64<float2> corners = new FixedList32<float2>();
            corners.Add(rect.c1 - lightSource);
            corners.Add(rect.c2 - lightSource);
            corners.Add(rect.c3 - lightSource);
            corners.Add(rect.c4 - lightSource);

            FixedList64<float2> cornersN = new FixedList32<float2>();
            foreach (float2 c in corners) {
                cornersN.Add(math.normalize(c));
            }

            float lowestDot = math.INFINITY;
            int l1 = -1;
            int l2 = -1;

            // The idea is to find the pair of corners which are the
            // furthest angle apart.
            for (int i = 0; i < 3; i++) {
                for (int j = i+1; j < 4; j++) {
                    float dot = math.dot(cornersN[i], cornersN[j]);
                    if (dot < lowestDot) {
                        lowestDot = dot;
                        l1 = i;
                        l2 = j;
                    }
                }
            }

            // Ensure they always have the same relative rotation. Improves
            // coherence of contact ids.
            if (Lin.Cross(cornersN[l1], cornersN[l2]) < 0) {
                var tmp = l1;
                l1 = l2;
                l2 = tmp;
            }


            // Finding the other 2 corners
            int o1 = 0;
            int o2 = 0;
            for (int i = 0; i < 4; i++) {
                if (i != l1 && i != l2) {
                    o1 = i;
                    break;
                }
            }
            for (int i = o1+1; i < 4; i++) {
                if (i != l1 && i != l2) {
                    o2 = i;
                    break;
                }
            }

            if (math.dot(cornersN[l1], cornersN[o1]) < math.dot(cornersN[l1], cornersN[o2])) {
                var tmp = o1;
                o1 = o2;
                o2 = tmp;
            }
            
            float reject1 = math.abs(Lin.Cross(cornersN[l1], corners[o1]));

            sg1 = new ShadowGeometry{
                contact1 = corners[l1] + lightSource,
                id1 = new float2(rect.id, l1).GetHashCode(),
            };
            if (reject1 < slop) {
                sg1.contact2 = corners[o1] + lightSource;
                sg1.id2 = new float2(rect.id, o1).GetHashCode();
            }

            float reject2 = math.abs(Lin.Cross(cornersN[l2], corners[o2]));

            sg2 = new ShadowGeometry{
                contact1 = corners[l2] + lightSource,
                id1 = new float2(rect.id, l2).GetHashCode()
            };
            if (reject2 < slop) {
                sg2.contact2 = corners[o2] + lightSource;
                sg2.id2 = new float2(rect.id, o2).GetHashCode();
            }
        }

        // IMPORTANT: This function extrapolates the subtracting shape so that
        // the raycast will always hit. This function should only be run on
        // shapes that are already known to overlap angularly.
        // Casts a ray with direction (lightOrigin towards shadowOrigin) from
        // shadowOrigin for distance shadowLength, and updates shadowLength
        // with the distance it traveled before hitting toSubtract.
        public static void ShadowSubtract(float2 lightOrigin, float2 shadowDirection, float shadowStart, ref float shadowEnd, Rect toSubtract) {
            
            float2 shadowOrigin = lightOrigin + shadowDirection*shadowStart;


            // Ensure the shadowNorm points from the rect center to the shadow
            float2 shadowNorm = Lin.Cross(shadowDirection, -1);
            float centerToShadowProj = math.dot(lightOrigin - toSubtract.pos, shadowNorm);

            if (centerToShadowProj < 0) {
                centerToShadowProj *= -1;
                shadowNorm *= -1;
            }

            //{
            //    float2 furthestVertex = toSubtract.FurthestVertexPoint(shadowNorm);
            //    float2 dir = Lin.Cross(math.normalize(furthestVertex - lightOrigin), -1);
            //    float dot = math.dot(furthestVertex - shadowOrigin, dir);
            //    if (math.abs(dot) < epsilon) {
            //        return;
            //    }
            //}


            float2 width = toSubtract.width;
            // make width point towards the shadow source
            if (math.dot(width, -shadowDirection) < 0) {
                width *= -1;
            }
            float widthProj = math.dot(width, shadowNorm);
            
            float2 height = toSubtract.height;
            // Make height point towards the shadow source
            if (math.dot(height, -shadowDirection) < 0) {
                height *= -1;
            }
            float heightProj = math.dot(height, shadowNorm);
            
            // Derived from: widthProj + heightProj*x = centerToShadowProj
            float heightMult = (centerToShadowProj - widthProj)/heightProj;
            
            // Derived from: heightProj + widthProj*x = centerToShadowProj
            float widthMult = (centerToShadowProj - heightProj)/widthProj;
            
            float2 p1 = toSubtract.pos + height + width*widthMult;
            float2 p2 = toSubtract.pos + width + height*heightMult;
            
            float2 intersection;
            
            if (heightMult <= 1 && widthMult <= 1) {
                if (math.dot(p1, shadowDirection) < math.dot(p2, shadowDirection)) {
                    intersection = p1;
                } else {
                    intersection = p2;
                }
            } else if (widthMult <= 1) {
                intersection = p1;
            } else if (heightMult <= 1) {
                intersection = p2;
            } else {
                // This part theoretically shouldn't happen, but floating point
                // imprecision might make this possible.
                intersection = (p1 + p2)/2;
            }

            shadowEnd = math.min(shadowEnd, math.dot(intersection - lightOrigin, shadowDirection));
        }

        // If the given shadow edge is partial (meaning at least one of its
        // endpoints is inside the rect), this function moves the offending
        // endpoints to the nearest points on the rect (along the shadow edge
        // line). It then returns whether this modified edge is intersecting.
        public static bool IsIntersectingShadowEdge(in Rect rect, float2 lightOrigin, float2 shadowDirection, float shadowStart, float shadowEnd, bool correctPartialIntersection) {

            float b0, b1;

            {
                float2 shadowNorm = Lin.Cross(shadowDirection, 1);

                var hits = new FixedList32<float>();

                int SideOf(float2 vertex) {
                    return (int)math.sign(math.dot(shadowNorm, vertex - lightOrigin));
                }
                float2 prevVertex = rect.GetVertex(-1);

                for (int i = 0; i < 4; i++) {
                    float2 currentVertex = rect.GetVertex(i);
                    if (SideOf(currentVertex) != SideOf(prevVertex)) {
                        hits.Add(Lin.IntersectionParameter(lightOrigin, shadowDirection, prevVertex, currentVertex - prevVertex));
                    }
                    prevVertex = currentVertex;
                }

                if (hits.Length == 0) {
                    return false;
                }
                b0 = hits[0];
                b1 = hits[0];
                for (int i = 1; i < hits.Length; i++) {
                    b0 = math.min(b0, hits[i]);
                    b1 = math.max(b1, hits[i]);
                }
            }

            {
                if (!correctPartialIntersection) {
                    return shadowEnd > b0 && shadowStart < b1;
                }
                if (shadowEnd <= b0 || shadowStart >= b1) {
                    return false;
                }
                void SetToClosest(ref float p, float r1, float r2) {
                    if (math.abs(p - r1) < math.abs(p - r2)) {
                        p = r1;
                    } else {
                        p = r2;
                    }
                }
                // At this point, shadowStart < b1 and shadowEnd > b0
                if (shadowStart > b0) { // if shadowStart is in the box
                    SetToClosest(ref shadowStart, b0, b1);
                }
                if (shadowEnd < b1) { // if shadowEnd is in the box
                    SetToClosest(ref shadowEnd, b0, b1);
                }

                if (shadowEnd <= b0 || shadowStart >= b1) {
                    return false;
                }

                return true;
            }
        }
    }
}

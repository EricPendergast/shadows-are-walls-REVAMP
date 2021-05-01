using Unity.Mathematics;
using Unity.Collections;
using UnityEngine;

namespace Physics.Math {

    public static partial class Geometry {
        public struct ShadowGeometry {
            public float2 contact1;
            public float2? contact2;
            public int id;
        }

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
                contact2 = reject1 < slop ? corners[o1] + lightSource : (float2?)null,
                id = new float2(rect.id, 1).GetHashCode()
            };

            float reject2 = math.abs(Lin.Cross(cornersN[l2], corners[o2]));

            sg2 = new ShadowGeometry{
                contact1 = corners[l2] + lightSource,
                contact2 = reject2 < slop ? corners[o2] + lightSource : (float2?)null,
                id = new float2(rect.id, 2).GetHashCode()
            };
        }

        // Casts a ray with direction (lightOrigin towards shadowOrigin) from
        // shadowOrigin for distance shadowLength, and updates shadowLength
        // with the distance it traveled before hitting toSubtract.
        public static void ShadowSubtract(float2 lightOrigin, float2 shadowDirection, float shadowStart, ref float shadowEnd, Rect shadowCastingShape, Box toSubtract) {
            var rect = toSubtract.ToRect();
            
            float2 shadowOrigin = lightOrigin + shadowDirection*shadowStart;

            // Special case: if the shadow origin is in the rect, pretend the
            // two shapes aren't intersecting. The "pretending" is achieved by
            // getting the minimal separation vector between the two shapes.
            if (rect.Contains(shadowOrigin)) {
                // TODO: We don't need the manifold contact points. If its a
                // performance problem (probably not), we can make a special
                // function that just gets the separation vector
                var manifoldNullable = GetIntersectData(rect, shadowCastingShape);
                if (manifoldNullable is Manifold manifold) {
                    float2 separation = manifold.normal * manifold.overlap;
                    if (math.dot(separation, shadowDirection) < 0) {
                        shadowEnd = math.min(shadowEnd, shadowStart);
                    } else {
                        shadowEnd = math.min(shadowEnd, shadowStart - 1);
                    }
                return;
                }
            }

            // Ensure the rayNorm points from the rect center to the shadowDirection
            float2 rayNorm = Lin.Cross(shadowDirection, -1);

            int closestVertex = rect.FurthestVertex(-shadowDirection);

            float centerToRayProj = math.dot(shadowOrigin - rect.pos, rayNorm);
            if (centerToRayProj < 0) {
                centerToRayProj *= -1;
                rayNorm *= -1;
            }

            float2 width = rect.width;
            // If corner has negative width term: (height-width) or (-height-width)
            if (closestVertex == 1 || closestVertex == 2) {
                width *= -1;
            }
            float widthProj = math.dot(width, rayNorm);
            
            float2 height = rect.height;
            // If corner has negative height term: (-height-width) or (-height+width)
            if (closestVertex == 2 || closestVertex == 3) {
                height *= -1;
            }
            float heightProj = math.dot(height, rayNorm);
            
            // Derived from: widthProj + heightProj*x = centerToRayProj
            float heightMult = (centerToRayProj - widthProj)/heightProj;
            
            // Derived from: heightProj + widthProj*x = centerToRayProj
            float widthMult = (centerToRayProj - heightProj)/widthProj;
            
            float2 p1 = rect.pos + height + width*widthMult;
            float2 p2 = rect.pos + width + height*heightMult;
            
            float2 intersection;
            
            if (math.abs(heightMult) <= 1 && math.abs(widthMult) <= 1) {
                if (math.dot(p1, shadowDirection) < math.dot(p2, shadowDirection)) {
                    intersection = p1;
                } else {
                    intersection = p2;
                }
            } else if (math.abs(widthMult) <= 1) {
                intersection = p1;
            } else if (math.abs(heightMult) <= 1) {
                intersection = p2;
            } else {
                return;
            }
            
            shadowEnd = math.min(shadowEnd, math.dot(intersection - lightOrigin, shadowDirection));
        }
    }
}

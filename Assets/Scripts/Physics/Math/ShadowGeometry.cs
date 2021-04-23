using Unity.Mathematics;
using Unity.Collections;
using UnityEngine;

namespace Physics.Math {

    public static partial class Geometry {
        public struct ShadowGeometry {
            public float2 contact1;
            public float2? contact2;
        }

        public static void CalculateShadowGeometry(Rect rect, float2 lightSource, float slop, out ShadowGeometry sg1, out ShadowGeometry sg2) {
            // TODO: rect.Contains is not implemented
            if (rect.Contains(lightSource)) {
                // TODO: Do something special
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
                contact2 = reject1 < slop ? corners[o1] + lightSource : (float2?)null
            };

            float reject2 = math.abs(Lin.Cross(cornersN[l2], corners[o2]));

            sg2 = new ShadowGeometry{
                contact1 = corners[l2] + lightSource,
                contact2 = reject2 < slop ? corners[o2] + lightSource : (float2?)null
            };
        }
    }
}

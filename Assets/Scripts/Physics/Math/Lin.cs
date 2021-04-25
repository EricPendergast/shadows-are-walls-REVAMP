using Unity.Mathematics;

namespace Physics.Math {
    public static class Lin {
        // Cross product uses the right hand rule: In the cross product
        // functions, curl your right hand from arg 1 to arg 2, and your thumb
        // points in the direction of the result, where towards yourself is
        // positive.

        public static float2 Cross(float2 a, float s) {
            return new float2(s * a.y, -s * a.x);
        }

        public static float2 Cross(float s, float2 a)
        {
            return new float2(-s * a.y, s * a.x);
        }

        public static float Cross(float2 v1, float2 v2) {
            return v1.x*v2.y - v2.x*v1.y;
        }

        public static float2 Reject(float2 a, float2 b) {
            return a - math.project(a, b);
        }

        public static float2 Rotate(float2 vec, float angle) {
            return math.mul(float2x2.Rotate(angle), vec);
        }
    }
}

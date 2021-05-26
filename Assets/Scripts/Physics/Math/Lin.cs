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

        public static bool IsFinite(float2 vec) {
            return math.isfinite(vec.x) && math.isfinite(vec.y);
        }

        // Gives the parameters u and v such that:
        //     p1 + u*p1Dir = p2 + v*p2Dir
        public static float2 IntersectionParameters(float2 p1, float2 p1Dir, float2 p2, float2 p2Dir) {
            float2 d = p2 - p1;
            float det = Lin.Cross(p2Dir, p1Dir);
            
            float u = Lin.Cross(p2Dir, d) / det;
            float v = Lin.Cross(p1Dir, d) / det;

            return new float2(u, v);
        }

        // Does the same as IntersectionParameters except it only returns the u
        // parameter.
        public static float IntersectionParameter(float2 p1, float2 p1Dir, float2 p2, float2 p2Dir) {
            float2 d = p2 - p1;
            float det = Lin.Cross(p2Dir, p1Dir);
            
            float u = Lin.Cross(p2Dir, d) / det;

            return u;
        }
    }
}

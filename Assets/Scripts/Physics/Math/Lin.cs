using Unity.Mathematics;

namespace Physics.Math {
    public static class Lin {

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
    }
}

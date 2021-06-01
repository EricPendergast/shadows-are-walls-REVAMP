using Unity.Mathematics;

namespace Physics.Math {
    public static class Springs {
        // Note that this damps velocity in all components (NOT just in the
        // component parallel to the spring).
        // Using the equation F = -k*x - b*v
        // Returns the force on p1
        public static float2 DampedSpringForce(float2 p1, float2 v1, float2 p2, float2 v2, float springLength, float k, float damping) {
            var p1ToP2 = p2 - p1;
            var dist = math.length(p1ToP2);
            var positionDifference = math.normalizesafe(p1ToP2)*(dist - springLength);
            var velocityDifference = v2 - v1;

            return k*(positionDifference) + damping*(velocityDifference);
        }
    }
}

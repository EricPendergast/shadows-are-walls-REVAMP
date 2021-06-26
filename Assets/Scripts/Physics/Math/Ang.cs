using Unity.Mathematics;

namespace Physics.Math {
    public struct AngleRange {
        public float rangeStart {get;}
        public float rangeLength {get;}
        public float rangeEnd => rangeStart + rangeLength;

        public AngleRange(float rangeStart, float rangeEnd) {
            this.rangeStart = rangeStart;
            this.rangeLength = Ang.CCWDistance(rangeStart, rangeEnd);
        }

        public float GetMinResolveOverlap(AngleRange other) {
            if (
                    this.Contains(other.rangeStart) || 
                    this.Contains(other.rangeEnd) || 
                    other.Contains(this.rangeStart) || 
                    other.Contains(this.rangeEnd)) {
                float resolve1 = Ang.SignedDistance(other.rangeEnd, this.rangeStart);
                float resolve2 = Ang.SignedDistance(other.rangeStart, this.rangeEnd);
                if (math.abs(resolve1) < math.abs(resolve2)) {
                    return resolve1;
                } else {
                    return resolve2;
                }
            } else {
                return 0;
            }
        }

        public bool Contains(float angle) {
            return Ang.CCWDistance(rangeStart, angle) <= rangeLength;
        }
    }

    public static class Ang {
        public static float ToRange0To2Pi(float angle) {
            float twoPi = 2*math.PI;
            angle = angle % twoPi;
            angle += twoPi;
            angle = angle % twoPi;
            return angle;
        }

        public static float ToRangeNegPiToPi(float angle) {
            return ToRange0To2Pi(angle + math.PI) - math.PI;
        }

        public static float SignedDistance(float angleFrom, float angleTo) {
            var ang = CCWDistance(angleFrom, angleTo);
            return ang > math.PI ? ang - 2*math.PI : ang;
        }
        public static float CCWDistance(float angleFrom, float angleTo) {
            float dist = angleTo - angleFrom;

            return ToRange0To2Pi(dist);
        }

        public static float SignedAngleOf(float2 vector) {
            var v1 = math.normalize(vector);
            var v2 = new float2(1, 0);
            float angle = math.acos(math.dot(v1, v2));
            if (Lin.Cross(v2, v1) < 0) {
                angle = -angle;
            }
            return angle;
        }
    }
}

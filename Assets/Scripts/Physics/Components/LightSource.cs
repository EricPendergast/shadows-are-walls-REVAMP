using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

public struct LightSource : IComponentData {
    public float2 pos;

    public float rot;
    public float inertia;
    public float aperture;

    public int minEdgeId;
    public int maxEdgeId;

    public float2 GetMinEdgeNorm() {
        return math.mul(float2x2.Rotate(rot - aperture/2), new float2(1, 0));
    }
    public float2 GetMaxEdgeNorm() {
        return math.mul(float2x2.Rotate(rot + aperture/2), new float2(1, 0));
    }

    public Rect GetMinEdgeRect() {
        return Rect.FromLineSegment(pos, pos + GetMinEdgeNorm()*20, minEdgeId);
    }

    public Rect GetMaxEdgeRect() {
        return Rect.FromLineSegment(pos, pos + GetMaxEdgeNorm()*20, maxEdgeId);
    }
}

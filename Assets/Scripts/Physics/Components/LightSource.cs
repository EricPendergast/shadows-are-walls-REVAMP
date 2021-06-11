using Unity.Entities;
using Unity.Mathematics;

using UnityEngine;

public struct LightSource : IComponentData {
    public float aperture;

    public int id;
    public int minEdgeId;
    public int maxEdgeId;

    public float2 GetLeadingEdgeNorm(float rot) {
        return math.mul(float2x2.Rotate(rot - aperture/2), new float2(1, 0));
    }
    public float2 GetTrailingEdgeNorm(float rot) {
        return math.mul(float2x2.Rotate(rot + aperture/2), new float2(1, 0));
    }

    // This is used by shaders
    public Matrix4x4 GetLightMatrix(Position pos) {
        return new Matrix4x4(
            (Vector2)pos.pos,
            (Vector2)GetLeadingEdgeNorm(pos.rot),
            (Vector2)GetTrailingEdgeNorm(pos.rot),
            Vector4.zero
        );
    }
}

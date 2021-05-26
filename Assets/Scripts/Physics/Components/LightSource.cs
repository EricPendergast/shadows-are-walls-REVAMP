using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

using UnityEngine;

public struct LightSource : IComponentData {
    public float2 pos;

    public float rot;
    public float aperture;

    public int id;
    public int minEdgeId;
    public int maxEdgeId;

    public float2 GetLeadingEdgeNorm() {
        return math.mul(float2x2.Rotate(rot - aperture/2), new float2(1, 0));
    }
    public float2 GetTrailingEdgeNorm() {
        return math.mul(float2x2.Rotate(rot + aperture/2), new float2(1, 0));
    }

    public float2 GlobalToLocal(float2 point) {
        return Lin.Rotate(point - pos, -rot);
    }

    // This is used by shaders
    public Matrix4x4 GetLightMatrix() {
        return new Matrix4x4(
            (Vector2)pos,
            (Vector2)GetLeadingEdgeNorm(),
            (Vector2)GetTrailingEdgeNorm(),
            Vector4.zero
        );
    }
}

using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

public struct LightEdge : IComponentData {
    public float2 pos;
    // TODO: Is this accurate enough? What kind of precision does
    // this allow over long distances?
    public float rot;
    public float inertia;

    public int id;

    public Rect ToRect() {
        float2 width = math.mul(float2x2.Rotate(rot), new float2(1, 0));
        // Rotate 90 degrees
        float2 height = Lin.Cross(width, -1)*.001f;
        width *= 100;
        return new Rect(pos + width, width, height, id);
    }
}

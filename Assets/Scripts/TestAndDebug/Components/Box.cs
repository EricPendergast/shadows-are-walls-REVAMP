
using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

public struct Box : IComponentData {
    public float2 pos;
    public float rot;

    public float mass;
    public float inertia;

    public float width;
    public float height;

    public int id;

    public Rect ToRect() {
        return Rect.FromWidthHeightAngle(pos, width, height, rot, id);
    }
}

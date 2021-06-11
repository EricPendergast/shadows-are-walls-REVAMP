
using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

public struct Box : IComponentData {
    public float width;
    public float height;

    public int id;

    public Rect ToRect(Position pos) {
        return Rect.FromWidthHeightAngle(pos.pos, width, height, pos.rot, id);
    }
}

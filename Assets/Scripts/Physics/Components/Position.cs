using Unity.Entities;
using Unity.Mathematics;
using Physics.Math;

public struct Position : IComponentData {
    public float2 pos;
    public float rot;

    public float2 GlobalToLocal(float2 point) {
        return Lin.Rotate(point - pos, -rot);
    }

    public float2 LocalToGlobal(float2 point) {
        return Lin.Rotate(point, rot) + pos;
    }

    public float2 GlobalDirectionToLocal(float2 direction) {
        return Lin.Rotate(direction , -rot);
    }

    public float2 LocalDirectionToGlobal(float2 direction) {
        return Lin.Rotate(direction, rot);
    }
}

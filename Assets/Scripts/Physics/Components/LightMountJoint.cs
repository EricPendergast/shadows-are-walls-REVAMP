using Unity.Entities;
using Unity.Mathematics;

public struct LightMountJoint : IComponentData {
    // The light is restricted to the space
    // starting at rangeStart and moving
    // counterclockwise (increasing angle) to
    // rangeEnd. 
    // rangeStart and rangeEnd are local to the
    // mount entity.
    public float rangeStart;
    public float rangeEnd;
    public Entity mount;

    public Physics.Math.AngleRange GetAngleRange(float mountRotation) {
        return new Physics.Math.AngleRange(rangeStart+mountRotation, rangeEnd+mountRotation);
    }
}


using Unity.Entities;

public struct LightTrackingJoint : IComponentData {
    public float trackSpeed;
    public float trackSoftness;
    public Entity toTrack;
}

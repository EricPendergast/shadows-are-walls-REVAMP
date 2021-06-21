using Unity.Entities;
using Unity.Mathematics;

public struct MouseComponent : IComponentData {
    public struct EntityGrabData {
        public Entity entity;
        public float2 grabLocalAnchor;
    }
    public EntityGrabData? grabData;
}

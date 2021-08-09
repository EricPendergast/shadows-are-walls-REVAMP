using Unity.Entities;

public struct PlayerRoot : IComponentData {
    // A reference to an entity with a DirectContactStore
    public Entity swappableDetector;
}

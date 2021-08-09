using Unity.Entities;

public struct PlayerBranch : IComponentData {
    // The entity with a PlayerRoot
    // component which the player
    // originally inhabited.
    public Entity playerRoot;
}

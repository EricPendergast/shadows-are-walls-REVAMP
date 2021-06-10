using Unity.Entities;

// An empty system which is used by MonoBehaviours to do things which
// only systems can do, such as
// - Getting a ComponentDataFromEntity object
// - Performing an entity query
public class DebugEmptySystem : SystemBase {
    protected override void OnUpdate() {}
}

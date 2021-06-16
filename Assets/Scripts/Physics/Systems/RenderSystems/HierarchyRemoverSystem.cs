using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;

[UpdateInGroup(typeof(SimulationSystemGroup))]
[UpdateBefore(typeof(TransformSystemGroup))]
public class HierarchyRemoverSystem : SystemBase {
    [WriteGroup(typeof(LocalToWorld))]
    private struct L2WBlocker : IComponentData {}

    protected override void OnUpdate() {
        World.EntityManager.RemoveComponent<Translation>(GetEntityQuery(typeof(IgnoreHierarchyTag), typeof(Translation)));
        World.EntityManager.RemoveComponent<Rotation>(GetEntityQuery(typeof(IgnoreHierarchyTag), typeof(Rotation)));
        World.EntityManager.RemoveComponent<NonUniformScale>(GetEntityQuery(typeof(IgnoreHierarchyTag), typeof(NonUniformScale)));

        World.EntityManager.AddComponent<LocalToWorld>(GetEntityQuery(typeof(IgnoreHierarchyTag)));
        World.EntityManager.AddComponent<L2WBlocker>(GetEntityQuery(typeof(IgnoreHierarchyTag)));
    }
}

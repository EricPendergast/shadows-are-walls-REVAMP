using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;

public class HierarchyRemoverSystem : SystemBase {


    protected override void OnUpdate() {
        var em = World.EntityManager;
        Entities
            .WithStructuralChanges()
            .WithAny<LocalToParent, Parent>()
            .WithAll<Box>()
            .ForEach((in Entity e) => {
                if (em.HasComponent<LocalToParent>(e)) {
                    em.RemoveComponent<LocalToParent>(e);
                }
                if (em.HasComponent<Parent>(e)) {
                    em.RemoveComponent<Parent>(e);
                }
            }).Run();
    }
}

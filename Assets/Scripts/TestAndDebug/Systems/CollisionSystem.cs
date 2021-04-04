using Unity.Entities;
using Unity.Transforms;
using Unity.Collections;
using Unity.Mathematics;

public class CollisionSystem : SystemBase {
    EntityQuery boxesQuery;

    private NativeList<Box> boxes;

    protected override void OnCreate() {
        boxes = new NativeList<Box>(100, Allocator.Persistent);
    }

    protected override void OnUpdate() {
        var boxes = this.boxes;
        while (boxes.Capacity < boxesQuery.CalculateEntityCount()) {
            boxes.Capacity *= 2;
        }

        
        Entities
            .WithName("StoreBoxes")
            .WithAll<Box>()
            .WithStoreEntityQueryInField(ref boxesQuery)
            .ForEach((int entityInQueryIndex, ref Box b) => {
                boxes[entityInQueryIndex] = b;
            }).ScheduleParallel();

    }
}

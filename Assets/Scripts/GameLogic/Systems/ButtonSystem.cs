using Unity.Entities;
using Unity.Mathematics;
using UnityEngine.InputSystem;
using UnityEngine;

[UpdateInGroup(typeof(PostContactGenerationGroup))]
public class ButtonSystem : SystemBase {
    protected override void OnUpdate() {
        Entities
        .WithStructuralChanges()
        .WithNone<DirectContactStore>()
        .WithAll<Button>()
        .ForEach((Entity e) => {
            World.EntityManager.AddBuffer<DirectContactStore>(e);
        }).Run();

        Entities.ForEach((ref DynamicBuffer<DirectContactStore> contacts, ref Button button) => {
            bool pressed = false;
            foreach (var contact in contacts) {
                //if (contact.normal.y > .1) {
                    pressed = true;
                //}
            }

            button.pressed = pressed;
        }).ScheduleParallel();
    }
}

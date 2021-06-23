using Unity.Entities;

using UnityEngine.InputSystem;


public class PlayerInputSystem : SystemBase {

    protected override void OnUpdate() {
        var keyboard = Keyboard.current;

        int lrDirection = 0;
        if (keyboard.aKey.isPressed) {
            lrDirection -= 1;
        }
        if (keyboard.dKey.isPressed) {
            lrDirection += 1;
        }

        Entities.ForEach(
        (ref PlayerComponent pc) => {
            pc.moveDirection = lrDirection;
        }).ScheduleParallel();
    }
}

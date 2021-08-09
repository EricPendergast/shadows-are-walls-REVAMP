using Unity.Entities;

using UnityEngine.InputSystem;


public class PlayerInputSystem : SystemBase {
    protected override void OnCreate() {
        if (!HasSingleton<PlayerControlInputs>()) {
            World.EntityManager.CreateEntity(typeof(PlayerControlInputs));
        }
    }

    protected override void OnUpdate() {
        var keyboard = Keyboard.current;

        int lrDirection = 0;
        if (keyboard.aKey.isPressed) {
            lrDirection -= 1;
        }
        if (keyboard.dKey.isPressed) {
            lrDirection += 1;
        }

        bool jump = keyboard.spaceKey.isPressed;

        bool swap = keyboard.eKey.wasPressedThisFrame;

        SetSingleton(new PlayerControlInputs {
            moveDirection = lrDirection,
            jumpPressed = jump,
            swapAttempted = swap
        });
    }
}

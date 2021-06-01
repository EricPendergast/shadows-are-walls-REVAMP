using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using UnityEngine.InputSystem;
using UnityEngine;

using Physics.Math;

[UpdateInGroup(typeof(SimulationSystemGroup))]
public class MouseDragSystem : SystemBase {
    float2? mousePrevPos;
    Entity? currentlySelected;
    float2 grabPoint;
    protected override void OnUpdate() {
        var mouse = Mouse.current;
        float2 mousePos = (Vector2)Camera.main.ScreenToWorldPoint(mouse.position.ReadValue());

        float2 mouseVel = mousePrevPos != null ? mousePrevPos.Value - mousePos : 0;
        //var leftPressed = mouse.leftButton.wasPressedThisFrame;
        //var leftReleased = mouse.leftButton.wasReleasedThisFrame;

        float dt = Time.DeltaTime;

        if (mouse.leftButton.wasPressedThisFrame) {
            currentlySelected = null;
            Entities
                .WithoutBurst()
                .ForEach((Entity e, ref Velocity v, ref Mass mass, ref Box box) => {
                if (box.ToRect().Contains(mousePos)) {
                    currentlySelected = e;
                    // TODO: Implement grabbing off center
                    grabPoint = mousePos - box.pos;
                }

            }).Run();
        }

        if(mouse.leftButton.wasReleasedThisFrame) {
            currentlySelected = null;
        }

        if (currentlySelected is Entity e) {
            var v = EntityManager.GetComponentData<Velocity>(e);
            var mass = EntityManager.GetComponentData<Mass>(e);
            var box = EntityManager.GetComponentData<Box>(e);

            if (math.isfinite(mass.mass)) {
                var accel = Springs.DampedSpringForce(
                    box.pos, v.vel,
                    mousePos, mouseVel,
                    0, // spring length
                    100, 20); // spring constants
                v.vel += accel*dt;
            } else {
                box.pos = mousePos;
            }

            EntityManager.SetComponentData(e, v);
            EntityManager.SetComponentData(e, mass);
            EntityManager.SetComponentData(e, box);
        }

        mousePrevPos = mousePos;
    }
}

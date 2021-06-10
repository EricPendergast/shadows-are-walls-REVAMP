using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using UnityEngine.InputSystem;
using UnityEngine;

using Physics.Math;

[AlwaysUpdateSystem]
[UpdateInGroup(typeof(SimulationSystemGroup))]
public class MouseDragSystem : SystemBase {
    float2? mousePrevPos;
    float2 grabPoint;

    protected override void OnCreate() {
        var em = World.EntityManager;

        if (!TryGetSingletonEntity<MouseComponent>(out Entity mouseEntity)) {
            mouseEntity = em.CreateEntity(typeof(MouseComponent));
        }

        em.AddComponentData(mouseEntity, 
            new Mass{inertia = math.INFINITY, mass= math.INFINITY}
        );
        em.AddComponentData(mouseEntity, 
            new Velocity()
        );
    }

    protected override void OnUpdate() {
        var em = World.EntityManager;
        var mouse = Mouse.current;
        //var leftPressed = mouse.leftButton.wasPressedThisFrame;
        //var leftReleased = mouse.leftButton.wasReleasedThisFrame;

        Entity mouseEntity = GetSingletonEntity<MouseComponent>();

        var mouseComponent = em.GetComponentData<MouseComponent>(mouseEntity);
        var mouseVelocity = em.GetComponentData<Velocity>(mouseEntity);

        {
            float2 currentMousePos = (Vector2)Camera.main.ScreenToWorldPoint(mouse.position.ReadValue());
            mouseVelocity.angVel = 0;
            mouseVelocity.vel = (currentMousePos - mouseComponent.pos)/Time.DeltaTime;
            mouseComponent.pos = currentMousePos;
        }

        if (mouse.leftButton.wasPressedThisFrame) {
            mouseComponent.grabData = null;
            Entities
                .WithoutBurst()
                .ForEach((Entity e, ref Velocity v, ref Mass mass, ref Box box) => {
                if (box.ToRect().Contains(mouseComponent.pos)) {
                    mouseComponent.grabData = new MouseComponent.EntityGrabData{
                        entity = e,
                        grabOffset = box.WorldVecToLocal(mouseComponent.pos - box.pos)
                    };
                }

            }).Run();
        }

        if(mouse.leftButton.wasReleasedThisFrame) {
            mouseComponent.grabData = null;
        }
        

        if (mouseComponent.grabData is MouseComponent.EntityGrabData gd) {
            em.AddComponentData(mouseEntity, new RevoluteJoint{
                e1 = mouseEntity,
                e2 = gd.entity,
                r1 = 0,
                r2 = gd.grabOffset,
                softness = 10,
                beta = .1f
            });
        } else {
            em.RemoveComponent<RevoluteJoint>(mouseEntity);
        }

        em.SetComponentData(mouseEntity, mouseComponent);
        em.SetComponentData(mouseEntity, mouseVelocity);

        //if (mouseComponent.grabData is MouseComponent.EntityGrabData g) {
        //    Debug.Log("Grabbed " + g.entity);
        //} else {
        //    Debug.Log("Not grabbing");
        //}
    }
}

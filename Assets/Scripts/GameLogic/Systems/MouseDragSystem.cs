using Unity.Entities;
using Unity.Mathematics;
using UnityEngine.InputSystem;
using UnityEngine;

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
        em.AddComponentData(mouseEntity, 
            new Position()
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
        var mousePosition = em.GetComponentData<Position>(mouseEntity);

        {
            float2 currentMousePos = (Vector2)Camera.main.ScreenToWorldPoint(mouse.position.ReadValue());
            mouseVelocity.angVel = 0;
            mouseVelocity.vel = (currentMousePos - mousePosition.pos)/Time.DeltaTime;
            mousePosition.pos = currentMousePos;
        }

        if (mouse.leftButton.wasPressedThisFrame) {
            mouseComponent.grabData = null;
            Entities
                .WithoutBurst()
                .ForEach((Entity e, in Velocity v, in Mass mass, in Box box, in Position pos) => {
                if (box.ToRect(pos).Contains(mousePosition.pos)) {
                    mouseComponent.grabData = new MouseComponent.EntityGrabData{
                        entity = e,
                        grabLocalAnchor = pos.GlobalToLocal(mousePosition.pos)
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
                localAnchor1 = float2.zero,
                localAnchor2 = gd.grabLocalAnchor,
                softness = 10f,
                beta = .1f
            });
        } else {
            em.RemoveComponent<RevoluteJoint>(mouseEntity);
        }

        em.SetComponentData(mouseEntity, mouseComponent);
        em.SetComponentData(mouseEntity, new Velocity());
        em.SetComponentData(mouseEntity, mousePosition);

        //if (mouseComponent.grabData is MouseComponent.EntityGrabData g) {
        //    Debug.Log("Grabbed " + g.entity);
        //} else {
        //    Debug.Log("Not grabbing");
        //}
    }
}

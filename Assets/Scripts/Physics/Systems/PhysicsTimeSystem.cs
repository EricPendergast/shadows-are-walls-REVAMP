using Unity.Entities;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup), OrderFirst=true)]
[UpdateAfter(typeof(BeginFixedStepSimulationEntityCommandBufferSystem))]
[AlwaysUpdateSystem]
public class PhysicsPreTimeSystem : SystemBase {
    // Need this because UnityEngine.Time.fixedTime does not update
    // with each tick of the ecs fixed loop.
    private struct FixedTimeCounter : IComponentData {
        public double elapsedTime;
    }

    protected override void OnCreate() {
        if (!HasSingleton<FixedTimeCounter>()) {
            var e = World.EntityManager.CreateEntity();
            World.EntityManager.AddComponentData(e,
                new FixedTimeCounter {
                    elapsedTime = 0
                }
            );
            World.EntityManager.SetName(e, "Fixed Time Counter");
        }
    }

    protected override void OnUpdate() {
        var timeCounter = GetSingleton<FixedTimeCounter>();
        float dt = UnityEngine.Time.fixedDeltaTime;

        World.PushTime(new Unity.Core.TimeData(
            elapsedTime: timeCounter.elapsedTime,
            deltaTime: dt
        ));

        timeCounter.elapsedTime += dt;
        SetSingleton(timeCounter);
    }
}

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup), OrderLast=true)]
[UpdateBefore(typeof(EndFixedStepSimulationEntityCommandBufferSystem))]
[AlwaysUpdateSystem]
public class PhysicsPostTimeSystem : SystemBase {
    protected override void OnUpdate() {
        World.PopTime();
    }
}

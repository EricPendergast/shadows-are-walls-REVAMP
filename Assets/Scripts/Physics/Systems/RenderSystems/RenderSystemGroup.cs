using Unity.Entities;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(CollisionSystem))]
[UpdateBefore(typeof(VelocityIntegrationSystem))]
public class RenderSystemGroup : ComponentSystemGroup {
}

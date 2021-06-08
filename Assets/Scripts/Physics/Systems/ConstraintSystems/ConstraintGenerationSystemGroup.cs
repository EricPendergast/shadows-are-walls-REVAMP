using Unity.Entities;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(GravitySystem))]
public class ConstraintGenerationSystemGroup : ComponentSystemGroup {
}

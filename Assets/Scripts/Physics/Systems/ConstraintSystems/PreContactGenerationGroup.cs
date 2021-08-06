using Unity.Entities;

[UpdateInGroup(typeof(ConstraintGenerationSystemGroup))]
[UpdateBefore(typeof(ContactGenerationGroup))]
public class PreContactGenerationGroup : ComponentSystemGroup {
}

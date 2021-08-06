using Unity.Entities;

[UpdateInGroup(typeof(ConstraintGenerationSystemGroup))]
[UpdateAfter(typeof(ContactGenerationGroup))]
public class PostContactGenerationGroup : ComponentSystemGroup {
}

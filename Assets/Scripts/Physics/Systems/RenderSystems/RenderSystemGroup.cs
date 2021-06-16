using Unity.Entities;
using Unity.Rendering;

[UpdateInGroup(typeof(PresentationSystemGroup))]
[UpdateBefore(typeof(RenderMeshSystemV2))]
public class RenderSystemGroup : ComponentSystemGroup {
}

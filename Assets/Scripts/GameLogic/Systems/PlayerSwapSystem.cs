using Unity.Entities;

using UnityEngine.InputSystem;


[UpdateAfter(typeof(PlayerInputSystem))]
public class PlayerSwapSystem : SystemBase {
    protected override void OnCreate() {
        RequireSingletonForUpdate<PlayerControlInputs>();
    }

    protected override void OnUpdate() {

        var controls = GetSingleton<PlayerControlInputs>();

        if (controls.swapAttempted) {
            var ecb = World.GetOrCreateSystem<EndSimulationEntityCommandBufferSystem>().CreateCommandBuffer().AsParallelWriter();
            
            var playerBranches = GetComponentDataFromEntity<PlayerBranch>(isReadOnly: true);
            Entities
            .WithAll<ActivePlayer, PlayerRoot>()
            .WithReadOnly(playerBranches)
            .ForEach((Entity playerRootEntity, int entityInQueryIndex, in DynamicBuffer<DirectContactStore> contacts) => {
                if (contacts.IsEmpty) {
                    return;
                }

                var swapTo = Entity.Null;
                foreach (var contact in contacts) {
                    if (playerBranches.HasComponent(contact.other)) {
                        swapTo = contact.other;
                        break;
                    }
                }

                if (swapTo == Entity.Null) {
                    return;
                }

                ecb.RemoveComponent<ActivePlayer>(entityInQueryIndex, playerRootEntity);
                ecb.AddComponent<ActivePlayer>(entityInQueryIndex, swapTo);
                ecb.SetComponent(entityInQueryIndex, swapTo, new PlayerBranch {
                    playerRoot = playerRootEntity
                });
            }).ScheduleParallel();

            Entities
            .WithAll<ActivePlayer>()
            .ForEach((Entity playerBranchEntity, int entityInQueryIndex, in PlayerBranch playerBranch) => {
                ecb.RemoveComponent<ActivePlayer>(entityInQueryIndex, playerBranchEntity);
                ecb.AddComponent<ActivePlayer>(entityInQueryIndex, playerBranch.playerRoot);
                ecb.SetComponent(entityInQueryIndex, playerBranchEntity, new PlayerBranch {
                    playerRoot = Entity.Null
                });
            }).ScheduleParallel();
        }
    }
}

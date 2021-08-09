using Unity.Entities;
using UnityEngine;

public class BranchPlayerAuthoring : MonoBehaviour, IConvertGameObjectToEntity {

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) {
        dstManager.AddComponentData(entity, 
            new PlayerBranch {
                playerRoot = Entity.Null
            }
        );

        dstManager.AddBuffer<DirectContactStore>(entity);
    }
}

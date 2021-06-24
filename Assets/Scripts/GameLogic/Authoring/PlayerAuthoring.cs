using Unity.Entities;
using UnityEngine;

public class PlayerAuthoring : MonoBehaviour, IConvertGameObjectToEntity {

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) {
        dstManager.AddComponentData(entity, 
            new PlayerComponent{
                moveDirection = 0
            }
        );

        dstManager.AddBuffer<DirectContactStore>(entity);
    }
}

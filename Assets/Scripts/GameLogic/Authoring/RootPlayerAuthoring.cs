using Unity.Entities;
using UnityEngine;

public class RootPlayerAuthoring : MonoBehaviour, IConvertGameObjectToEntity {

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) {
        dstManager.AddComponentData(entity, 
            new PlayerRoot { // TODO: Have the proper swappable detector
                swappableDetector = Entity.Null
            }
        );

        dstManager.AddComponent<ActivePlayer>(entity);
        dstManager.AddBuffer<DirectContactStore>(entity);
    }
}

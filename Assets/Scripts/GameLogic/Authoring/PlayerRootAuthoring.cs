using Unity.Entities;
using UnityEngine;

public class PlayerRootAuthoring : MonoBehaviour, IConvertGameObjectToEntity {

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) {
        Entity swappableDetector = conversionSystem.CreateAdditionalEntity(this);

        dstManager.AddBuffer<DirectContactStore>(swappableDetector);
        dstManager.AddComponent<Box>(swappableDetector);
        dstManager.AddComponent<Position>(swappableDetector);
        dstManager.AddComponent<IsTrigger>(swappableDetector);

        dstManager.AddComponentData(entity, 
            new PlayerRoot {
                swappableDetector = swappableDetector
            }
        );

        dstManager.AddComponent<ActivePlayer>(entity);
        dstManager.AddBuffer<DirectContactStore>(entity);
        dstManager.AddBuffer<ShadowContactStore>(entity);
    }
}

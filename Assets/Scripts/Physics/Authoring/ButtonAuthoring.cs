using Unity.Entities;
using Unity.Mathematics;

using UnityEngine;

public class ButtonAuthoring : MonoBehaviour, IConvertGameObjectToEntity {
    public GameObject controlled;

    public float pressedAngVel;
    public float unpressedAngVel;

    public float softness;

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) {

        dstManager.AddComponentData(entity, 
            new Button {pressed = false}
        );

        if (controlled != null) {
            dstManager.AddComponentData(entity, 
                new ButtonLightController {
                    controlledLight = conversionSystem.GetPrimaryEntity(controlled),
                    pressedAngVel = math.radians(pressedAngVel),
                    unpressedAngVel = math.radians(unpressedAngVel),
                    softness = softness
                }
            );
        }
    }
}

using Unity.Entities;
using UnityEngine;

public class BoxAuthoring : MonoBehaviour, IConvertGameObjectToEntity {
    public Vector2 velocity;
    public float angularVelocity;

    public float mass = 1;
    public float inertia = .1f;

    public float width = 1;
    public float height = 1;

    public float gravityScale = 1;

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) {
        dstManager.AddComponentData(entity, 
            new Box {
                pos = (Vector2)transform.position,
                rot = transform.eulerAngles.z*Mathf.Deg2Rad,
                vel = velocity,
                angVel = angularVelocity*Mathf.Deg2Rad,
                mass = mass,
                inertia = inertia,
                width = transform.localScale.x*width,
                height = transform.localScale.y*height
            });

        if (gravityScale != 1) {
            dstManager.AddComponentData(entity,
                new GravityScale {gravityScale = gravityScale}
            );
        }

    }
}



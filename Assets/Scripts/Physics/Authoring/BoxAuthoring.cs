using Unity.Entities;
using UnityEngine;

public class BoxAuthoring : MonoBehaviour, IConvertGameObjectToEntity {
    public enum BoxType {
        HitShadowsObject,
        TransparentObject,
        OpaqueObject
    }
    public BoxType boxType;

    public Vector2 velocity;
    public float angularVelocity;

    public float mass = 1;
    public bool fixPosition = false;
    public bool fixRotation = false;

    public float width = 1;
    public float height = 1;

    public float gravityScale = 1;

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) {
        float w = transform.localScale.x*width;
        float h = transform.localScale.y*height;
        var inertia = fixRotation ? Mathf.Infinity : this.mass*(w*w + h*h)/12;
        var mass = fixPosition ? Mathf.Infinity : this.mass;
        var gravityScale = fixPosition ? 0 : this.gravityScale;

        dstManager.AddComponentData(entity, 
            new Box {
                width = w,
                height = h,
                id = Random.Range(1, int.MaxValue)
            });

        dstManager.AddComponentData(entity,
            new Position {
                pos = (Vector2)transform.position,
                rot = transform.eulerAngles.z*Mathf.Deg2Rad,
            });

        dstManager.AddComponentData(entity, 
            new Velocity {
                vel = velocity,
                angVel = angularVelocity*Mathf.Deg2Rad,
            });
        dstManager.AddComponentData(entity, 
            new Mass {
                mass = mass,
                inertia = inertia
            });

        if (gravityScale != 1) {
            dstManager.AddComponentData(entity,
                new GravityScale {gravityScale = gravityScale}
            );
        }

        if (boxType == BoxType.HitShadowsObject) {
            dstManager.AddComponentData(entity, new HitShadowsObject());
        }

        if (boxType == BoxType.OpaqueObject) {
            dstManager.AddComponentData(entity, new OpaqueObject());
        }
    }
}



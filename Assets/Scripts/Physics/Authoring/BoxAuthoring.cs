using Unity.Entities;
using UnityEngine;

public class BoxAuthoring : MonoBehaviour, IConvertGameObjectToEntity {
    public enum BoxType {
        HitShadowsObject,
        TransparentObject,
        OpaqueObject
    }
    public BoxType boxType;
    public bool isTrigger;

    public Vector2 velocity;
    public float angularVelocity;

    public float mass = 1;
    public bool fixPosition = false;
    public bool fixRotation = false;

    public float width = 1;
    public float height = 1;

    public float gravityScale = 1;

    public float friction = .75f;

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) {
        var fixRotation = isTrigger ? true : this.fixRotation;
        var fixPosition = isTrigger ? true : this.fixPosition;

        float w = GetWidth();
        float h = GetHeight();
        var inertia = fixRotation ? Mathf.Infinity : this.mass*(w*w + h*h)/12;
        var mass = fixPosition ? Mathf.Infinity : this.mass;
        var gravityScale = fixPosition ? 0 : this.gravityScale;

        dstManager.AddComponentData(entity, GetBox());

        dstManager.AddComponentData(entity,
            GetPosition());

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

        dstManager.AddComponentData(entity, new IgnoreHierarchyTag());

        dstManager.AddComponentData(entity, new Friction{friction = friction});

        if (isTrigger) {
            dstManager.AddComponentData(entity, new IsTrigger());
        }

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

    private Position GetPosition() {
        return new Position {
            pos = (Vector2)transform.position,
            rot = transform.eulerAngles.z*Mathf.Deg2Rad,
        };
    }

    private float GetWidth() {
        return transform.localScale.x*width;
    }

    private float GetHeight() {
        return transform.localScale.y*height;
    }

    private Box GetBox() {
        return new Box {
            width = GetWidth(),
            height = GetHeight(),
            id = GetHashCode()
        };
    }

    public Physics.Math.Rect GetRect() {
        return new Box{width = GetWidth(), height = GetHeight()}.ToRect(GetPosition());
    }
}



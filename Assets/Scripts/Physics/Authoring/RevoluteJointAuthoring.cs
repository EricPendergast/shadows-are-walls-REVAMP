using System.Collections.Generic;

using Unity.Entities;
using Unity.Mathematics;

using UnityEngine;
using UnityEditor;

public class RevoluteJointAuthoring : MonoBehaviour, IConvertGameObjectToEntity {
    public GameObject obj1;
    public GameObject obj2;

    public float beta;
    public float softness;

    public float2 offset;

    public Entity e;

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) {

        float2 x1 = (Vector2)obj1.transform.position;
        float2 x2 = (Vector2)obj2.transform.position;

        conversionSystem.GetPrimaryEntity(obj1);
        dstManager.AddComponentData(entity, new RevoluteJoint{
            e1 = conversionSystem.GetPrimaryEntity(obj1),
            e2 = conversionSystem.GetPrimaryEntity(obj2),
            r1 = GetR1(),
            r2 = GetR2(),
            beta = beta,
            softness = softness
        });
    }

    Vector2 WorldToLocal(GameObject obj, Vector2 point) {
        point -= (Vector2)obj.transform.position;
        point = obj.transform.InverseTransformDirection(point);
        return point;
    }

    Vector2 GetR1() {
        return WorldToLocal(obj1, (Vector2)transform.position - (Vector2)offset/2);
    }
    Vector2 GetR2() {
        return WorldToLocal(obj2, (Vector2)transform.position + (Vector2)offset/2);
    }

    void OnDrawGizmos() {
        Vector2 x1 = obj1.transform.position;
        Vector2 x2 = obj2.transform.position;

        Gizmos.DrawSphere(x1 + GetR1(), .1f);
        Gizmos.DrawRay(x1, GetR1());
        Gizmos.DrawSphere(x2 + GetR2(), .1f);
        Gizmos.DrawRay(x2, GetR2());

        Gizmos.color = Color.red;
        Gizmos.DrawLine(x1 + GetR1(), x2 + GetR2());
    }
}

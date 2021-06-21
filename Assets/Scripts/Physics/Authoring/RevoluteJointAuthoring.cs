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
            localAnchor1 = GetLocalAnchor1(),
            localAnchor2 = GetLocalAnchor2(),
            beta = beta,
            softness = softness
        });
    }

    Vector2 WorldToLocal(GameObject obj, Vector2 point) {
        point -= (Vector2)obj.transform.position;
        point = obj.transform.InverseTransformDirection(point);
        return point;
    }

    Vector2 GetLocalAnchor1() {
        return WorldToLocal(obj1, GetMount1());
    }
    Vector2 GetLocalAnchor2() {
        return WorldToLocal(obj2, GetMount2());
    }

    Vector2 GetMount1() {
        return (Vector2)transform.position - (Vector2)offset/2;
    }

    Vector2 GetMount2() {
        return (Vector2)transform.position + (Vector2)offset/2;
    }

    void OnDrawGizmos() {
        Vector2 x1 = obj1.transform.position;
        Vector2 x2 = obj2.transform.position;

        Vector2 m1 = GetMount1();
        Vector2 m2 = GetMount2();

        Gizmos.DrawSphere(m1, .1f);
        Gizmos.DrawLine(x1, m1);
        Gizmos.DrawSphere(m2, .1f);
        Gizmos.DrawLine(x2, m2);

        Gizmos.color = Color.red;
        Gizmos.DrawLine(m1, m2);
    }
}

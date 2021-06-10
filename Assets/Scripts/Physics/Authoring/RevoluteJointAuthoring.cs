using Unity.Entities;
using Unity.Mathematics;

using UnityEngine;

public class RevoluteJointAuthoring : MonoBehaviour, IConvertGameObjectToEntity {
    public GameObject obj1;
    public GameObject obj2;

    public float beta;
    public float softness;


    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) {

        float2 x1 = (Vector2)obj1.transform.position;
        float2 x2 = (Vector2)obj2.transform.position;

        Vector2 WorldToLocal(GameObject obj, Vector2 point) {
            point -= (Vector2)obj.transform.position;
            point = obj.transform.InverseTransformDirection(point);
            return point;
        }

        conversionSystem.GetPrimaryEntity(obj1);
        dstManager.AddComponentData(entity, new RevoluteJoint{
            e1 = conversionSystem.GetPrimaryEntity(obj1),
            e2 = conversionSystem.GetPrimaryEntity(obj2),
            r1 = WorldToLocal(obj1, transform.position),
            r2 = WorldToLocal(obj2, transform.position),
            beta = beta,
            softness = softness
        });
    }

    void OnDrawGizmos() {
        Gizmos.DrawSphere(transform.position, .1f);
        Gizmos.DrawLine(obj1.transform.position, transform.position);
        Gizmos.DrawLine(obj2.transform.position, transform.position);
    }
}

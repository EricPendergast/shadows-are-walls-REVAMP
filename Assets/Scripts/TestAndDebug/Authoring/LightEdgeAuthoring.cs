using Unity.Entities;
using UnityEngine;

public class LightEdgeAuthoring : MonoBehaviour, IConvertGameObjectToEntity {
    public float angularVelocity;

    public float inertia = .1f;


    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) {
        dstManager.AddComponentData(entity, 
            new LightEdge {
                pos = (Vector2)transform.position,
                rot = transform.eulerAngles.z*Mathf.Deg2Rad,
                inertia = inertia,
                id = Random.Range(1, int.MaxValue)
            });

        dstManager.AddComponentData(entity, 
            new Velocity {
                vel = 0,
                angVel = angularVelocity*Mathf.Deg2Rad,
            });

        dstManager.AddComponentData(entity,
            new GravityScale {gravityScale = 0}
        );
    }

    void OnDrawGizmos() {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(transform.position, .1f);
        Gizmos.DrawRay(transform.position, transform.rotation*Vector2.right*20);
    }
}



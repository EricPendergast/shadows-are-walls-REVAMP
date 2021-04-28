using Unity.Entities;
using UnityEngine;
using Unity.Rendering;

public class LightAuthoring : MonoBehaviour, IConvertGameObjectToEntity {

    public float angularVelocity;
    public float inertia = .1f;
    public float aperture = 40;

    public Material material;

    void Awake() {
        gameObject.AddComponent<MeshFilter>().mesh = new Mesh();
        gameObject.AddComponent<MeshRenderer>().material = material;
    }

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) {
        dstManager.AddComponentData(entity, 
            new LightSource {
                pos = (Vector2)transform.position,
                rot = transform.eulerAngles.z*Mathf.Deg2Rad,
                inertia = inertia,
                aperture = aperture * Mathf.Deg2Rad,
                maxEdgeId = Random.Range(1, int.MaxValue),
                minEdgeId = Random.Range(1, int.MaxValue),
                id = Random.Range(1, int.MaxValue),
            }
        );

        dstManager.AddComponentData(entity, 
            new Velocity {
                vel = 0,
                angVel = angularVelocity*Mathf.Deg2Rad,
            }
        );

        dstManager.AddComponentData(entity,
            new GravityScale {gravityScale = 0}
        );

        Entity edge1 = conversionSystem.CreateAdditionalEntity(gameObject);
        Entity edge2 = conversionSystem.CreateAdditionalEntity(gameObject);
    }

    void OnDrawGizmos() {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(transform.position, .1f);

        float rot = transform.eulerAngles.z;
        Gizmos.DrawRay(transform.position, Quaternion.Euler(0,0,rot+aperture/2)*Vector2.right*20);
        Gizmos.DrawRay(transform.position, Quaternion.Euler(0,0,rot-aperture/2)*Vector2.right*20);
    }
}

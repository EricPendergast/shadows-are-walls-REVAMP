using Unity.Entities;
using UnityEngine;
using Unity.Mathematics;

using Random = UnityEngine.Random;

using Physics.Math;

public class LightAuthoring : MonoBehaviour, IConvertGameObjectToEntity {

    public float angularVelocity;
    public float inertia = .1f;
    public float aperture = 40;

    public GameObject mount;
    public float rangeStart;
    public float rangeEnd;

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) {
        dstManager.AddComponentData(entity, GetLightSource());
        dstManager.AddComponentData(entity, GetLightPosition());

        dstManager.AddComponentData(entity, 
            new Velocity {
                vel = 0,
                angVel = angularVelocity*Mathf.Deg2Rad,
            }
        );
        dstManager.AddComponentData(entity, 
            new Mass {
                mass = math.INFINITY,
                inertia = inertia
            }
        );

        dstManager.AddComponentData(entity,
            new GravityScale {gravityScale = 0}
        );

        dstManager.AddComponentData(entity, new IgnoreHierarchyTag());

        if (mount != null) {
            dstManager.AddComponentData(entity,
                new LightMountJoint{
                    mount = conversionSystem.GetPrimaryEntity(mount),
                    rangeStart = math.radians(rangeStart),
                    rangeEnd = math.radians(rangeEnd)
                }
            );
        }
    }

    public Position GetLightPosition() {
        return new Position {
            pos = (Vector2)transform.position,
            rot = transform.eulerAngles.z*Mathf.Deg2Rad,
        };
    }

    public LightSource GetLightSource() {
        return new LightSource {
                aperture = aperture * Mathf.Deg2Rad,
                maxEdgeId = Random.Range(1, int.MaxValue),
                minEdgeId = Random.Range(1, int.MaxValue),
                id = Random.Range(1, int.MaxValue),
            };
    }

    void OnDrawGizmos() {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(transform.position, .1f);

        float rot = transform.eulerAngles.z;
        Gizmos.DrawRay(transform.position, Quaternion.Euler(0,0,rot+aperture/2)*Vector2.right*20);
        Gizmos.DrawRay(transform.position, Quaternion.Euler(0,0,rot-aperture/2)*Vector2.right*20);

        if (mount != null) {
            Gizmos.DrawRay(transform.position, Quaternion.Euler(0,0, rangeStart) * mount.transform.right);
            Gizmos.DrawRay(transform.position, Quaternion.Euler(0,0, rangeEnd) * mount.transform.right);
        }
    }
}

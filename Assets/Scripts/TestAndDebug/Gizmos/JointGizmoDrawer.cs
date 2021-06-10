using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;

public class JointGizmoDrawer : MonoBehaviour {
    public bool enable = false;
    public Color color = Color.gray;
    public Color colorBetween = Color.red;
    public float drawRadius = .1f;

    void OnDrawGizmos() {
        if (enabled && Application.isPlaying) {
            var em = World.DefaultGameObjectInjectionWorld.EntityManager;
            
            var jointSystem = World.DefaultGameObjectInjectionWorld.GetOrCreateSystem<RevoluteJointSystem>();
            var mouses = jointSystem.GetComponentDataFromEntity<MouseComponent>();
            var boxes = jointSystem.GetComponentDataFromEntity<Box>();

            float2 GetPos(Entity e) {
                if (mouses.HasComponent(e)) {
                    return mouses[e].pos;
                } else {
                    return boxes[e].pos;
                }
            }


            foreach (var joint in jointSystem.GetManifoldsForDebug()) {
                var x1 = (Vector2)GetPos(joint.e1);
                var x2 = (Vector2)GetPos(joint.e2);
                var m1 = x1 + (Vector2)joint.r1;
                var m2 = x2 + (Vector2)joint.r2;

                Gizmos.color = color;
                Gizmos.DrawLine(x1, m1);
                Gizmos.DrawSphere(m1, drawRadius);

                Gizmos.color = colorBetween;
                Gizmos.DrawLine(m1, m2);

                Gizmos.color = color;
                Gizmos.DrawLine(x2, m2);
                Gizmos.DrawSphere(m2, drawRadius);

            }
        }
    }

}

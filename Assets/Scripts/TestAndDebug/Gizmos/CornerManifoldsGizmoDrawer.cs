using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Physics.Math;
using Unity.Mathematics;

public class CornerManifoldsGizmoDrawer : MonoBehaviour {
    public bool enable = true;
    public float drawRadius = .05f;
    void OnDrawGizmos() {
        if (Application.isPlaying && enable) {
            var world = World.DefaultGameObjectInjectionWorld;

            var set = new HashSet<int>();

            foreach (var manifold in world.GetOrCreateSystem<ShadowConstraintSystem>().GetCornerManifoldsForDebug()) {
                int id = Mathf.Abs(manifold.contactIdOnBox);
                // Indicates there is a duplicate contact. This should never happen
                float m = set.Contains(id) ? 5 : 1;
                set.Add(id);
                
                Gizmos.color = new Color(
                        (id % 4591 % 256)/256.0f, 
                        (id % 5347 % 256)/265.0f,
                        (id % 3797 % 256)/265.0f);

                void DrawPoint(Vector2 point) {
                    Gizmos.DrawSphere(point, drawRadius);
                    Gizmos.DrawRay(point, (Vector2)manifold.n);
                }

                DrawPoint(manifold.p1);
                DrawPoint(manifold.p2);
                DrawPoint(manifold.p);
            }
        }
    }
}



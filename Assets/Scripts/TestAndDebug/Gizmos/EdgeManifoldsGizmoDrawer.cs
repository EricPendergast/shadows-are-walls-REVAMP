using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Physics.Math;

public class EdgeManifoldsGizmoDrawer : MonoBehaviour {
    public bool enable = true;
    public float drawRadius = .05f;
    void OnDrawGizmos() {
        if (Application.isPlaying && enable) {
            var world = World.DefaultGameObjectInjectionWorld;

            var set = new HashSet<int>();

            foreach (var manifold in world.GetOrCreateSystem<ShadowConstraintSystem>().GetEdgeManifoldsForDebug()) {
                Gizmos.color = Color.red;
                Gizmos.DrawRay((Vector2)manifold.p, (Vector2)manifold.n);
                int id = Mathf.Abs(manifold.contactIdOn2.GetHashCode());
                // Indicates there is a duplicate contact. This should never happen
                Debug.Assert(!set.Contains(id), "Duplicate edge manifolds with id " + id);
                float m = set.Contains(id) ? 5 : 1;
                set.Add(manifold.contactIdOn2);
                
                Gizmos.color = new Color(
                        (id % 4591 % 256)/256.0f, 
                        (id % 5347 % 256)/265.0f,
                        (id % 3797 % 256)/265.0f);
                Gizmos.DrawSphere((Vector2)manifold.p, drawRadius*m);
            }
        }
    }
}



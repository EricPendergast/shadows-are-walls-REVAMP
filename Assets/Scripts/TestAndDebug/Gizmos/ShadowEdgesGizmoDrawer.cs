using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

public class ShadowEdgesGizmoDrawer : MonoBehaviour {
    void OnDrawGizmos() {
        if (Application.isPlaying) {
            var world = World.DefaultGameObjectInjectionWorld;

            var ids = new HashSet<int>();

            foreach (var shadowEdge in world.GetOrCreateSystem<ShadowEdgeGenerationSystem>().GetShadowEdgesForDebug()) {
                int id = shadowEdge.id;
                if (ids.Contains(id)) {
                    Debug.LogError("Duplicate shadow edges with id: " + id);
                }
                ids.Add(id);

                id = math.abs(id);

                Gizmos.color = new Color(
                        (id % 4591 % 256)/256.0f, 
                        (id % 5347 % 256)/265.0f,
                        (id % 3797 % 256)/265.0f);

                Gizmos.DrawLine((Vector2)shadowEdge.contact1, (Vector2)(shadowEdge.contact1 + math.normalize(shadowEdge.contact1 - shadowEdge.lightSource)*shadowEdge.length));

                Gizmos.DrawSphere((Vector2)shadowEdge.contact1, .05f);
                if (shadowEdge.contact2 != null) {
                    Gizmos.color = Color.green;
                    Gizmos.DrawSphere((Vector2)shadowEdge.contact2, .05f);
                }
            }
        }
    }
}



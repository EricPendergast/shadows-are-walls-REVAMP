using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

public class ShadowEdgesGizmoDrawer : MonoBehaviour {
    public bool enable = true;
    void OnDrawGizmos() {
        if (Application.isPlaying && enable) {
            var world = World.DefaultGameObjectInjectionWorld;

            var ids = new HashSet<int>();

            foreach (var shadowEdge in world.GetOrCreateSystem<ShadowEdgeGenerationSystem>().GetShadowEdgesForDebug()) {
                void CheckId(int id) {
                    if (ids.Contains(id)) {
                        Debug.LogError("Duplicate shadow edges with id: " + id);
                    }
                    ids.Add(id);
                }
                CheckId(shadowEdge.id1);
                if (shadowEdge.id2 != null) {
                    CheckId(shadowEdge.id2.Value);
                }

                void DrawMount(float2 point, int id) {
                    id = math.abs(id);

                    Gizmos.color = new Color(
                            (id % 4591 % 256)/256.0f, 
                            (id % 5347 % 256)/265.0f,
                            (id % 3797 % 256)/265.0f);
                    Gizmos.DrawSphere((Vector2)point, .05f);
                }

                Gizmos.color = Color.red;
                Gizmos.DrawLine((Vector2)shadowEdge.mount1, (Vector2)shadowEdge.endpoint);

                DrawMount(shadowEdge.mount1, shadowEdge.id1);
                if (shadowEdge.mount2 != null) {
                    DrawMount(shadowEdge.mount2.Value, shadowEdge.id2.Value);
                }
            }
        }
    }
}



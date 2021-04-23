using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;

public class ShadowEdgesGizmoDrawer : MonoBehaviour {
    void OnDrawGizmos() {
        if (Application.isPlaying) {
            var world = World.DefaultGameObjectInjectionWorld;

            foreach (var shadowEdge in world.GetOrCreateSystem<ShadowEdgeGenerationSystem>().GetShadowEdgesForDebug()) {
                Gizmos.color = Color.red;
                Gizmos.DrawLine((Vector2)shadowEdge.contact1, (Vector2)shadowEdge.lightSource);
                Gizmos.DrawSphere((Vector2)shadowEdge.contact1, .05f);
                if (shadowEdge.contact2 != null) {
                    Gizmos.color = Color.green;
                    Gizmos.DrawSphere((Vector2)shadowEdge.contact2, .05f);
                }
            }
        }
    }
}



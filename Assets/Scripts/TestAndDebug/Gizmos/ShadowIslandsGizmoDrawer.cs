using Unity.Entities;
using System.Collections.Generic;
using UnityEngine;

using Corner = CornerCalculator.Corner;

public class ShadowIslandsGizmoDrawer : MonoBehaviour {
    public bool enable = true;
    void OnDrawGizmos() {
        if (Application.isPlaying && enable) {
            var world = World.DefaultGameObjectInjectionWorld;


            List<Corner> corners = world.GetOrCreateSystem<ShadowEdgeGenerationSystem>().GetShadowIslandsForDebug();
            if (corners.Count == 0) {
                return;
            }
            Corner islandStartCorner = corners[0];

            for (int i = 0; i < corners.Count-1; i++) {
                Corner current = corners[i];
                if (islandStartCorner.isNull) {
                    islandStartCorner = current;
                }
                Corner next = corners[i+1];
                if (next.isNull) {
                    next = islandStartCorner;
                    i++;
                    islandStartCorner = Corner.Null;
                }

                Gizmos.color = Color.red;
                Gizmos.DrawLine((Vector2)current.point, (Vector2)next.point);
            }
        }
    }
}



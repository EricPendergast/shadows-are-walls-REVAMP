using Unity.Entities;
using System.Collections.Generic;
using UnityEngine;

using Corner = CornerCalculator.Corner;

public class ShadowIslandsGizmoDrawer : MonoBehaviour {
    public bool enable = true;
    [Range(-1, 15)]
    public int onlyDrawIslandsWithIndex = -1;

    void OnDrawGizmos() {
        if (Application.isPlaying && enable) {
            var world = World.DefaultGameObjectInjectionWorld;


            List<Corner> corners = world.GetOrCreateSystem<ShadowEdgeGenerationSystem>().GetShadowIslandsForDebug();
            if (corners.Count == 0) {
                return;
            }
            Corner islandStartCorner = corners[0];
            int islandIndex = 0;

            for (int i = 0; i < corners.Count-1; i++) {
                Corner current = corners[i];
                if (islandStartCorner.isNull) {
                    islandStartCorner = current;
                    islandIndex++;
                }
                Corner next = corners[i+1];
                if (next.isNull) {
                    next = islandStartCorner;
                    i++;
                    islandStartCorner = Corner.Null;
                }

                if (onlyDrawIslandsWithIndex == -1 || islandIndex == onlyDrawIslandsWithIndex) {
                    Gizmos.color = Color.red;
                    Gizmos.DrawLine((Vector2)current.point, (Vector2)next.point);
                }
            }
        }
    }
}



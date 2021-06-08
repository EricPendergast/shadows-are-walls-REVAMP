using Unity.Mathematics;
using Unity.Entities;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

using Corner = ShadowCornerCalculator.Corner;

public class ShadowIslandsGizmoDrawer : MonoBehaviour {
    public bool enable = true;
    [Range(-1, 15)]
    public int onlyDrawIslandsWithIndex = -1;

    [Range(-5, 20)]
    public int onlyDrawEdgesWithIndex = -5;
    public bool drawEdgeLabels = true;
    public Color islandsColor = Color.red;

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

                bool drawingIsland = onlyDrawIslandsWithIndex == -1 || islandIndex == onlyDrawIslandsWithIndex;
                bool drawingEdge = onlyDrawEdgesWithIndex == -5 || current.nextEdge == onlyDrawEdgesWithIndex;

                if (drawingIsland && drawingEdge) {
                    Gizmos.color = islandsColor;
                    Gizmos.DrawLine((Vector2)current.point, (Vector2)next.point);
                    if (drawEdgeLabels) {
                        float2 midpoint = (current.point + next.point)/2;
                        GUI.color = islandsColor;
                        Handles.Label((Vector2)midpoint, current.nextEdge.ToString());
                    }
                }
            }
        }
    }
}



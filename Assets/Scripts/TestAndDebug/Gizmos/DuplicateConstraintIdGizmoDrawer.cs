
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using Physics.Math;

using CornerMountTuple = ShadowCornerCalculator.Outputs.CornerMountTuple;

public class DuplicateConstraintIdGizmoDrawer : MonoBehaviour {
    public bool enable;

    void OnDrawGizmos() {
        if (Application.isPlaying && enable) {
            var world = World.DefaultGameObjectInjectionWorld;

            var dict = new MultiDict<int, CornerMountTuple>();

            foreach (var cornerMountTuple in world.GetOrCreateSystem<ShadowEdgeGenerationSystem>().GetCornerMountsForDebug()) {
                dict.Add(cornerMountTuple.partialConstraint.id, cornerMountTuple);
            }

            foreach (var kv in dict) {
                var id = kv.Key;
                var list = kv.Value;
                if (list.Count > 1) {
                    Debug.Log("Duplicate corner contact found with id: " + id);
                    foreach (var tup in list) {
                        var manifold = tup.m;
                        float2 point = 0;
                        int numPoints = 0;
                        void AddIfFinite(float2 p) {
                            if (Lin.IsFinite(p)) {
                                point += p;
                                numPoints++;
                            }
                        }
                        AddIfFinite(manifold.p);
                        AddIfFinite(manifold.p1);
                        AddIfFinite(manifold.p2);

                        Debug.Assert(numPoints > 0);
                        point /= numPoints;

                        Gizmos.DrawSphere((Vector2)point, .2f);
                    }
                }
            }
        }
    }
}


using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using Physics.Math;

using CornerMountTuple = ShadowCornerCalculator.Outputs.CornerMountTuple;
using EdgeMountTuple = ShadowCornerCalculator.Outputs.EdgeMountTuple;

public class DuplicateConstraintIdGizmoDrawer : MonoBehaviour {
    public bool enable;
    public float drawRadius = .2f;

    void OnDrawGizmos() {
        if (Application.isPlaying && enable) {
            CheckCornerConstraints();
            CheckEdgeConstraints();
        }
    }

    void CheckCornerConstraints() {
        var dict = new MultiDict<int, CornerMountTuple>();

        var world = World.DefaultGameObjectInjectionWorld;

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

                    Gizmos.DrawSphere((Vector2)point, drawRadius);
                }
            }
        }
    }
    void CheckEdgeConstraints() {
        var dict = new MultiDict<int, EdgeMountTuple>();

        var world = World.DefaultGameObjectInjectionWorld;

        foreach (var edgeMountTuple in world.GetOrCreateSystem<ShadowEdgeGenerationSystem>().GetEdgeMountsForDebug()) {
            dict.Add(edgeMountTuple.partialConstraint.id, edgeMountTuple);
        }

        foreach (var kv in dict) {
            var id = kv.Key;
            var list = kv.Value;
            if (list.Count > 1) {
                Debug.Log("Duplicate edge contact found with id: " + id);
                foreach (var tup in list) {
                    Gizmos.DrawSphere((Vector2)tup.m.p, drawRadius);
                }
            }
        }
    }
}

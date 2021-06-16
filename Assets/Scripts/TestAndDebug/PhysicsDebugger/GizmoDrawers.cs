using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;
using Physics.Math;

public partial class PhysicsDebugger {
    public Color color = Color.gray;
    public Color colorBetween = Color.red;
    public float drawRadius = .1f;

    private List<RevoluteJointManifold> jointManifolds;

    private void SaveGizmosState() {
        var jointSystem = World.DefaultGameObjectInjectionWorld.GetExistingSystem<RevoluteJointSystem>();
        if (jointSystem != null) {
            jointManifolds = jointSystem.GetManifoldsForDebug();
        }
    }

    private void OnDrawGizmos() {
        if (!IsDebugging()) {
            SaveGizmosState();
        }
        var helperSystem = World.DefaultGameObjectInjectionWorld.GetOrCreateSystem<HelperSystem>();
        if (jointManifolds != null) {
            var positions = helperSystem.GetComponentDataFromEntity<Position>();
            foreach (var joint in jointManifolds) {
                Vector2 x1 = positions[joint.e1].pos;
                Vector2 x2 = positions[joint.e2].pos;
                Vector2 m1 = x1 + (Vector2)joint.r1;
                Vector2 m2 = x2 + (Vector2)joint.r2;

                Gizmos.color = color;
                Gizmos.DrawLine(x1, m1);
                Gizmos.DrawSphere(m1, drawRadius);

                Gizmos.color = colorBetween;
                Gizmos.DrawLine(m1, m2);

                Gizmos.color = color;
                Gizmos.DrawLine(x2, m2);
                Gizmos.DrawSphere(m2, drawRadius);
            }
        }
    }
}

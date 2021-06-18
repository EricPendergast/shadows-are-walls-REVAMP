using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;
using Physics.Math;

public partial class PhysicsDebugger {
    public enum DrawMode {
        drawGizmosBeforeDebug,
        drawGizmosAfterDebug
    }
    public DrawMode drawMode = DrawMode.drawGizmosAfterDebug;
    [SerializeField]
    public IDebuggableConstraint.DrawGizmosSettings gizmoSettings = new IDebuggableConstraint.DrawGizmosSettings{
        color = Color.gray,
        springStretchColor = Color.red,
        drawRadius = .1f
    };

    List<IDebuggableConstraint> constraintsAfterDebug = new List<IDebuggableConstraint>();

    private void OnDrawGizmos() {
        if (!Application.isPlaying) {
            return;
        }
        if (drawMode == DrawMode.drawGizmosAfterDebug) {
            foreach (var dc in constraintsAfterDebug) {
                dc.DrawGizmos(gizmoSettings);
            }
        } else {
            if (ConstraintsUpToDate()) {
                foreach (var dc in constraints) {
                    dc.DrawGizmos(gizmoSettings);
                }
            } else {
                var gizmoConstraints = new List<IDebuggableConstraint>();
                StoreConstraints(gizmoConstraints);
                foreach (var dc in gizmoConstraints) {
                    dc.DrawGizmos(gizmoSettings);
                }
            }
        }
    }
}

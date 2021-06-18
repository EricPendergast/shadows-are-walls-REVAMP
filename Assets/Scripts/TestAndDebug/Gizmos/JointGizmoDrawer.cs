using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;
using Physics.Math;

public class JointGizmoDrawer : MonoBehaviour {
    public Color color = Color.gray;
    public Color colorBetween = Color.red;
    public float drawRadius = .1f;

    public bool debugJacobians = false;
    public float fractionToResolve = 1;

    void Update() {}
    //void OnDrawGizmos() {
    //    if (enabled && Application.isPlaying) {
    //        var world = World.DefaultGameObjectInjectionWorld;
    //        var em = world.EntityManager;
    //        
    //        var jointSystem = World.DefaultGameObjectInjectionWorld.GetOrCreateSystem<RevoluteJointSystem>();
    //        var masses = jointSystem.GetComponentDataFromEntity<Mass>();
    //        var mouses = jointSystem.GetComponentDataFromEntity<MouseComponent>();
    //        var boxes = jointSystem.GetComponentDataFromEntity<Box>();
    //        var velocities = jointSystem.GetComponentDataFromEntity<Velocity>();
    //
    //        var positions = jointSystem.GetComponentDataFromEntity<Position>();
    //
    //
    //        foreach (var joint in jointSystem.GetManifoldsForDebug()) {
    //            Vector2 x1 = positions[joint.e1].pos;
    //            Vector2 x2 = positions[joint.e2].pos;
    //            Vector2 m1 = x1 + (Vector2)joint.r1;
    //            Vector2 m2 = x2 + (Vector2)joint.r2;
    //
    //            Gizmos.color = color;
    //            Gizmos.DrawLine(x1, m1);
    //            Gizmos.DrawSphere(m1, drawRadius);
    //
    //            Gizmos.color = colorBetween;
    //            Gizmos.DrawLine(m1, m2);
    //
    //            Gizmos.color = color;
    //            Gizmos.DrawLine(x2, m2);
    //            Gizmos.DrawSphere(m2, drawRadius);
    //
    //
    //            if (debugJacobians) {
    //                var jointChanged = joint;
    //                jointChanged.beta = 1;
    //                jointChanged.softness = 0;
    //                var constraint = new TwoWayTwoDOFConstraint(jointChanged, masses, 1);
    //
    //                Velocity v1 = new Velocity();
    //                Velocity v2 = new Velocity();
    //                constraint.ApplyImpulses(ref v1, ref v2, dt:1);
    //                void Draw(Entity e, Velocity v, float2 r) {
    //                    v.vel *= fractionToResolve;
    //                    v.angVel *= fractionToResolve;
    //                    if (boxes.HasComponent(e)) {
    //                        Gizmos.color = Color.red;
    //                        JacobianGizmoDrawer.DrawWireframeBoxGizmo(world, e, v.vel, v.angVel);
    //                        r = Lin.Rotate(r, v.angVel);
    //                        Gizmos.DrawRay((Vector2)(positions[e].pos+v.vel), (Vector2)r);
    //                    }
    //                }
    //
    //                Draw(joint.e1, v1, joint.r1);
    //                Draw(joint.e2, v2, joint.r2);
    //            }
    //        }
    //    }
    //}

}

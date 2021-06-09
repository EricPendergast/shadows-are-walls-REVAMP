using Unity.Mathematics;
using Unity.Entities;
using UnityEngine;
using Physics.Math;
using System.Collections.Generic;

public class JacobianGizmoDrawer : MonoBehaviour {
    public Color color;
    public bool enable;
    public int numIterations = 1;

    private class MassGetter : SystemBase {
        protected override void OnUpdate() {}
    }
    //private class DebugWireframeDrawer : SystemBase {
    //    protected override void OnCreate() {
    //
    //    }
    //    protected override void OnUpdate() {
    //
    //    }
    //}

    private static void DrawWireframeShadowEdgeGizmo(World w, Entity e, float2 edgeDir, float2 source, float2 mount, float2 posOffset, float rotOffset) {
        if (w.EntityManager.HasComponent<Box>(e)) {
            DrawWireframeOpaqueEdgeGizmo(w, e, 
                source: source,
                mount: mount,
                posOffset: posOffset,
                rotOffset: rotOffset
            );
        }
        if (w.EntityManager.HasComponent<LightSource>(e)) {
            DrawWireframeLightEdgeGizmo(w, e,
                edgeDir: edgeDir,
                source: source,
                rotOffset: rotOffset
            );
        }
    }
    private static void DrawWireframeLightEdgeGizmo(World w, Entity e, float2 edgeDir, float2 source, float rotOffset) {
        var em = w.EntityManager;
        var ls = em.GetComponentData<LightSource>(e);

        Gizmos.DrawRay((Vector2)ls.pos, (Vector2)Lin.Rotate(edgeDir, rotOffset)*50);
    }

    private static void DrawWireframeOpaqueEdgeGizmo(World w, Entity e, float2 source, float2 mount, float2 posOffset, float rotOffset) {

        var em = w.EntityManager;

        DrawWireframeBoxGizmo(w, e, posOffset, rotOffset);

        var box = em.GetComponentData<Box>(e);

        mount = Lin.Rotate(mount - box.pos, rotOffset) + box.pos + posOffset;

        var dir = math.normalize(mount - source);

        Gizmos.DrawRay((Vector2)mount, (Vector2)dir*50);
    }

    private static void DrawWireframeBoxGizmo(World w, Entity e) {
        DrawWireframeBoxGizmo(w, e, 0, 0);
    }

    private static void DrawWireframeBoxGizmo(World w, Entity e, float2 posOffset, float angOffset) {
        Box box = w.EntityManager.GetComponentData<Box>(e);
        box.pos += posOffset;
        box.rot += angOffset;
        var rect = box.ToRect();
        Gizmos.DrawLine((Vector2)rect.c1, (Vector2)rect.c2);
        Gizmos.DrawLine((Vector2)rect.c2, (Vector2)rect.c3);
        Gizmos.DrawLine((Vector2)rect.c3, (Vector2)rect.c4);
        Gizmos.DrawLine((Vector2)rect.c4, (Vector2)rect.c1);
    }

    void OnDrawGizmos() {
        if (Application.isPlaying && enable) {
            Gizmos.color = color;
            var world = World.DefaultGameObjectInjectionWorld;

            var massGetter = world.GetOrCreateSystem<MassGetter>();
            var masses = massGetter.GetComponentDataFromEntity<Mass>();

            var velocities = new Dictionary<Entity, Velocity>();
            
            foreach (var tup in world.GetOrCreateSystem<ShadowConstraintSystem>().GetCornerMountsForDebug()) {
                var (manifold, mount1, mount2, mount3, pConstraint) = tup;

                // Recalculating pConstraint to have bias such that a complete resolution occurs
                var prototype = new ThreeWayPenConstraint.Partial.Prototype(in manifold, beta:1, delta_slop:0);
                if (mount3 != null) {
                    Debug.Log("JacobianGizmoDrawer does not support triple shadow edge constraints");
                    //pConstraint = new ShadowCornerConstraint.Partial(prototype, mount1, mount2, mount3.Value, float2.zero, manifold);
                } else {
                    pConstraint = new ThreeWayPenConstraint.Partial(prototype, mount1, mount2, pConstraint.e3, manifold);
                }
                var constraint = new ThreeWayPenConstraint(pConstraint, masses, dt: 1);

                Velocity GetVelocity(Entity e) {
                    if (velocities.TryGetValue(e, out var vel)) {
                        return vel;
                    } else {
                        return new Velocity();
                    }
                }

                void SetVelocity(Entity e, Velocity v) {
                    velocities[e] = v;
                }

                velocities.Clear();

                Velocity v1;
                Velocity v2;
                Velocity v3;

                for (int i = 0; i < numIterations; i++) {
                    v1 = GetVelocity(constraint.e1);
                    v2 = GetVelocity(constraint.e2);
                    v3 = GetVelocity(constraint.e3);

                    constraint.ApplyImpulses(ref v1, ref v2, ref v3, dt: 1);

                    velocities.Clear();
                    SetVelocity(constraint.e1, v1);
                    SetVelocity(constraint.e2, v2);
                    SetVelocity(constraint.e3, v3);
                }

                v1 = GetVelocity(constraint.e1);  
                v2 = GetVelocity(constraint.e2);
                v3 = GetVelocity(constraint.e3);

                DrawWireframeShadowEdgeGizmo(world, constraint.e1, 
                    source: manifold.x1,
                    edgeDir: manifold.d1,
                    mount: mount1.point,
                    posOffset: v1.vel,
                    rotOffset: v1.angVel
                );

                DrawWireframeShadowEdgeGizmo(world, constraint.e2, 
                    source: manifold.x2,
                    edgeDir: manifold.d2,
                    mount: mount2.point,
                    posOffset: v2.vel,
                    rotOffset: v2.angVel
                );

                DrawWireframeBoxGizmo(world, constraint.e3, v3.vel, v3.angVel);
            }
        }
    }
}

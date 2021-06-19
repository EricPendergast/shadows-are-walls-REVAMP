using System.Collections.Generic;

using Unity.Entities;
using Unity.Mathematics;
using Unity.Rendering;

using UnityEditor;
using UnityEngine;

[ExecuteAlways]
public partial class PhysicsDebugger : MonoBehaviour {
    public int sequentialImpulseIterations = 1;
    public bool performWarmStarting = true;
    public bool resolveConstraintsCompletely = false;
    public float applyVelocityScale = 1;


    [SerializeField]
    private List<IDebuggableConstraint> constraints = new List<IDebuggableConstraint>();
    [HideInInspector]
    private double lastLoadOfConstraints = -1;

    private static float DeltaTime() {
        return World.DefaultGameObjectInjectionWorld.GetExistingSystem<CollisionSystem>().DeltaTime();
    }

    private class HelperSystem : SystemBase {
        private Dictionary<Entity, Position> positions = new Dictionary<Entity, Position>();
        private Dictionary<Entity, Velocity> velocities = new Dictionary<Entity, Velocity>();
        protected override void OnUpdate() {}

        public void ApplyVelocities(float applyVelocityScale, float dt) {
            Entities.WithoutBurst().ForEach(
                (ref Position p, in Velocity v, in Entity e) => {
                    p.pos += v.vel * dt * applyVelocityScale;
                    p.rot += v.angVel * dt * applyVelocityScale;
                }
            ).Run();
        }

        public void SavePhysicsState() {
            positions.Clear();
            Entities.WithoutBurst().ForEach((in Position p, in Entity e) => {
                positions[e] = p;
            }).Run();
            velocities.Clear();
            Entities.WithoutBurst().ForEach((in Velocity v, in Entity e) => {
                velocities[e] = v;
            }).Run();
        }

        public void RestorePhysicsState() {
            Entities.WithoutBurst().ForEach((ref Position p, in Entity e) => {
                p = positions[e];
            }).Run();
            Entities.WithoutBurst().ForEach((ref Velocity v, in Entity e) => {
                v = velocities[e];
            }).Run();
        }
    }

    private bool CanRun() {
        var world = World.DefaultGameObjectInjectionWorld;
        var collisionSystem = world.GetExistingSystem<CollisionSystem>();
        return Application.isPlaying && EditorApplication.isPaused && enabled && collisionSystem != null;
    }

    private bool IsDebugging() {
        return CanRun() && ConstraintsUpToDate();
    }

    private static void StoreConstraints(List<IDebuggableConstraint> storeInto) {
        // This retrieves the constraints by running the constraint
        // generation group, and then getting the constraints from the
        // collision system.
        var world = World.DefaultGameObjectInjectionWorld;

        var shadowConstraintSystem = world.GetOrCreateSystem<ShadowConstraintSystem>();
        shadowConstraintSystem.Update();
        var masses = shadowConstraintSystem.GetComponentDataFromEntity<Mass>();

        float dt = DeltaTime();

        storeInto.Clear();

        storeInto.AddRange(
            world.GetExistingSystem<RevoluteJointSystem>()?.GetDebuggableConstraints());

        storeInto.AddRange(
            world.GetExistingSystem<DirectConstraintSystem>()?.GetDebuggableConstraints());

        //constraints = new List<IConstraintWrapper>();
        //
        //foreach (CornerMountTuple tup in shadowConstraintSystem.GetCornerMountsForDebug()) {
        //    var c = new ThreeWayPenConstraint(tup.partialConstraint, masses, dt: dt);
        //    var cFullResolve = new ThreeWayPenConstraint(tup.partialConstraint, masses, dt: dt, beta: 1, delta_slop: 0);
        //    constraints.Add(new ConstraintWrapper<ThreeWayPenConstraint, float>(c, cFullResolve));
        //}
        //
        //foreach (EdgeMountTuple tup in shadowConstraintSystem.GetEdgeMountsForDebug()) {
        //    var c = new TwoWayPenConstraint(tup.partialConstraint, masses, dt: dt);
        //    var cFullResolve = new TwoWayPenConstraint(tup.partialConstraint, masses, dt: dt, beta: 1, delta_slop: 0);
        //    constraints.Add(new ConstraintWrapper<TwoWayPenConstraint, float>(c, cFullResolve));
        //}
    }

    private void UpdateConstraints() {
        var collisionSystem = World.DefaultGameObjectInjectionWorld.GetExistingSystem<CollisionSystem>();
        lastLoadOfConstraints = collisionSystem.Time.ElapsedTime;

        PhysicsDebugger.StoreConstraints(constraints);
    }

    private bool ConstraintsUpToDate() {
        var collisionSystem = World.DefaultGameObjectInjectionWorld.GetExistingSystem<CollisionSystem>();
        return lastLoadOfConstraints == collisionSystem.Time.ElapsedTime;
    }

    void OnDisable() {
        Render();
    }

    void Render() {
        var world = World.DefaultGameObjectInjectionWorld;

        world?.GetExistingSystem<RenderSystemGroup>()?.Update();
        world?.GetExistingSystem<RenderMeshSystemV2>()?.Update();
    }

    void RenderConstraintResults() {
        var world = World.DefaultGameObjectInjectionWorld;
        var helperSystem = world.GetOrCreateSystem<HelperSystem>();

        helperSystem.SavePhysicsState();
        world.GetOrCreateSystem<GravitySystem>().Update();
        ApplyConstraints();
        Render();
        StoreConstraints(constraintsAfterDebug);
        helperSystem.RestorePhysicsState();

        UnityEditorInternal.InternalEditorUtility.RepaintAllViews();
    }

    void ApplyConstraints() {
        var world = World.DefaultGameObjectInjectionWorld;
        var collisionSystem = world.GetExistingSystem<CollisionSystem>();
        var helperSystem = world.GetOrCreateSystem<HelperSystem>();

        //world.GetExistingSystem<GravitySystem>().Update();

        float dt = DeltaTime();

        var constraints = new List<IDebuggableConstraint>();
        foreach (var c in this.constraints) {
            constraints.Add(c.Clone());
        }
        if (resolveConstraintsCompletely) {
            foreach (var c in constraints) {
                c.SetConstants(new IDebuggableConstraint.Constants{
                    beta = 1,
                    delta_slop = 0,
                    softness = 0
                });
            }
        }

        var velocities = helperSystem.GetComponentDataFromEntity<Velocity>(false);

        if (performWarmStarting) {
            foreach (var dc in constraints) {
                collisionSystem.DebugApplyWarmStart(dc.GetConstraint(), velocities, dt);
            }
        }

        for (int i = 0; i < sequentialImpulseIterations; i++) {
            foreach (var dc in constraints) {
                dc.GetConstraint().ApplyImpulses(velocities, dt);
            }
        }

        helperSystem.ApplyVelocities(applyVelocityScale, dt);
    }

    [CustomEditor(typeof(PhysicsDebugger))]
    private class MyEditor : Editor {
        
        public override void OnInspectorGUI() {
            var t = target as PhysicsDebugger;

            if (t.IsDebugging()) {
                GUILayout.Label("Debugging", EditorStyles.boldLabel);
                if (GUILayout.Button("Reload Constraints")) {
                    t.UpdateConstraints();
                }
            } else if (t.CanRun()) {
                if (GUILayout.Button("Load Constraints and Start Debugging")) {
                    t.UpdateConstraints();
                }
            } else {
                GUILayout.Label("Not debugging because:", EditorStyles.boldLabel);
                if (!t.enabled) {
                    GUILayout.Label("Component is disabled.", EditorStyles.boldLabel);
                }
                if (!Application.isPlaying) {
                    GUILayout.Label("Scene is not in play mode.", EditorStyles.boldLabel);
                }
                if (!EditorApplication.isPaused) {
                    GUILayout.Label("Editor is not paused.", EditorStyles.boldLabel);
                }
                if (World.DefaultGameObjectInjectionWorld.GetExistingSystem<CollisionSystem>() == null) {
                    GUILayout.Label("CollisionSystem is not created.", EditorStyles.boldLabel);
                }
            }
            DrawDefaultInspector();

            if(GUI.changed) {
                if (t.IsDebugging()) {
                    t.RenderConstraintResults();
                }
                UnityEditorInternal.InternalEditorUtility.RepaintAllViews();
            }
        }

        public override bool RequiresConstantRepaint() {
            return true;
        }
    }
}

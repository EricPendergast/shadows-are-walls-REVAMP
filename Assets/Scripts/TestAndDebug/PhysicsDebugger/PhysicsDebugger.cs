using System.Collections.Generic;

using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Rendering;

using UnityEditor;
using UnityEngine;

[ExecuteAlways]
public class PhysicsDebugger : MonoBehaviour {
    public int sequentialImpulseIterations = 1;
    public bool performWarmStarting = true;
    public bool resolveConstraintsCompletely = false;
    public float applyVelocityScale = 1;
    public float2 shift;


    public List<IConstraint> constraints = new List<IConstraint>();
    [HideInInspector]
    private double lastLoadOfConstraints = -1;

    [HideInInspector]
    private bool autoRendering = false;
    [HideInInspector]
    private bool renderOnce = false;


    private class HelperSystem : SystemBase {
        private Dictionary<Entity, Position> positions = new Dictionary<Entity, Position>();
        private Dictionary<Entity, Velocity> velocities = new Dictionary<Entity, Velocity>();
        protected override void OnUpdate() {}

        public void Do(float2 shift) {
            Entities
                .ForEach((ref Position pos) => {
                    pos.pos += shift;
                }).Run();
        }

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

    private void LoadConstraints() {
        // This retrieves the constraints by running the constraint
        // generation group, and then getting the constraints from the
        // collision system.
        var world = World.DefaultGameObjectInjectionWorld;
        var constraintGatherer = world.GetExistingSystem<ConstraintGatherSystem>();

        constraintGatherer.ClearConstraintBuffers();

        world.GetExistingSystem<ConstraintGenerationSystemGroup>().Update();

        constraints = new List<IConstraint>();
        constraints.AddRange(constraintGatherer.DebugIterAllConstraints());

        constraintGatherer.ClearConstraintBuffers();

        lastLoadOfConstraints = constraintGatherer.Time.ElapsedTime;
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
        world?.GetExistingSystem<TransformSystemGroup>()?.Update();
        world?.GetExistingSystem<RenderMeshSystemV2>()?.Update();
    }

    void RenderConstraintResults() {
        var world = World.DefaultGameObjectInjectionWorld;
        var helperSystem = world.GetOrCreateSystem<HelperSystem>();

        helperSystem.SavePhysicsState();
        world.GetOrCreateSystem<GravitySystem>().Update();
        //helperSystem.Do(shift);
        ApplyConstraints();
        Render();
        helperSystem.RestorePhysicsState();

        SceneView.lastActiveSceneView.Repaint();
    }

    void ApplyConstraints() {
        var world = World.DefaultGameObjectInjectionWorld;
        var collisionSystem = world.GetExistingSystem<CollisionSystem>();
        var helperSystem = world.GetOrCreateSystem<HelperSystem>();

        var constraints = new List<IConstraint>();
        foreach (var c in this.constraints) {
            constraints.Add(c.Clone());
        }

        float dt = collisionSystem.DeltaTime();

        if (resolveConstraintsCompletely) {
            foreach (var constraint in constraints) {
                constraint.DebugMultiplyBias(1 / constraint.GetBeta());
            }
        }

        var velocities = helperSystem.GetComponentDataFromEntity<Velocity>(false);

        if (performWarmStarting) {
            foreach (var constraint in constraints) {
                collisionSystem.DebugApplyWarmStart(constraint, velocities, dt);
            }
        }

        for (int i = 0; i < sequentialImpulseIterations; i++) {
            foreach (var constraint in constraints) {
                constraint.ApplyImpulses(velocities, dt);
            }
        }

        helperSystem.ApplyVelocities(applyVelocityScale, dt);
    }

    //void OnDrawGizmos() {
    //    if (Application.isPlaying && enable) {
    //        var world = World.DefaultGameObjectInjectionWorld;
    //
    //        var set = new HashSet<int>();
    //
    //        Debug.LogError("ContactGizmoDrawer is currently broken");
    //
    //        world.GetOrCreateSystem<HelperSystem>().Do(shift);
    //        world.GetExistingSystem<RenderMeshSystemV2>().Update();
    //
    //        //foreach (var contact in world.GetOrCreateSystem<CollisionSystem>().GetContactsForDebug()) {
    //        //    // Indicates there is a duplicate contact. This should never happen
    //        //    float m = set.Contains(contact.id) ? 5 : 1;
    //        //    set.Add(contact.id);
    //        //    
    //        //    Gizmos.color = Color.red;
    //        //    //Gizmos.DrawSphere((Vector2)contact.contact, .01f);
    //        //    Gizmos.DrawRay((Vector2)contact.contact, (Vector2)contact.normal);
    //        //
    //        //    int id = Mathf.Abs(contact.id.GetHashCode());
    //        //
    //        //    Gizmos.color = new Color(
    //        //            (id % 4591 % 256)/256.0f, 
    //        //            (id % 5347 % 256)/265.0f,
    //        //            (id % 3797 % 256)/265.0f);
    //        //    Gizmos.DrawSphere((Vector2)contact.contact, .05f*m);
    //        //}
    //    }
    //}

    [CustomEditor(typeof(PhysicsDebugger))]
    private class MyEditor : Editor {
        
        public void OnSceneGUI() {
            var t = target as PhysicsDebugger;
            Debug.Log("Scene GUI");
            Debug.Log("t.renderOnce = " + t.renderOnce);

            if (t.CanRun() && (t.autoRendering || t.renderOnce)) {
                Debug.Log("Rendering");
                t.RenderConstraintResults();
                t.renderOnce = false;
            }
            //var tr = t.transform;
            //var pos = tr.position;
            //// display an orange disc where the object is
            //var color = new Color(1, 0.8f, 0.4f, 1);
            //Handles.color = color;
            //Handles.DrawWireDisc(pos, tr.up, 1.0f);
            //// display object "value" in scene
            //GUI.color = color;
            //Handles.Label(pos, t.value.ToString("F1"));
        }

        public override void OnInspectorGUI() {
            var t = target as PhysicsDebugger;
            //if (t.ShouldRun()) {
            //    t.RenderConstraintResults();
            //    GUILayout.Label("Running State: Running", EditorStyles.boldLabel);
            //} else {
            //    GUILayout.Label("Running State: Not Running", EditorStyles.boldLabel);
            //}

            if (t.CanRun()) {
                if (t.ConstraintsUpToDate()) {
                    if (GUILayout.Button("Reload Constraints")) {
                        t.LoadConstraints();
                    }
                    t.autoRendering = GUILayout.Toggle(t.autoRendering, "Auto Render");
                    if (!t.autoRendering) {
                        if (GUILayout.Button("Render")) {
                            Debug.Log("Rendering once");
                            Debug.Log("Rendering once");
                            t.RenderConstraintResults();
                            t.renderOnce = true;
                        }
                    }
                } else {
                    if (GUILayout.Button("Load Constraints")) {
                        t.LoadConstraints();
                    }
                }
            } else {
                GUILayout.Label("Not Running. Scene must be playing and paused in order to run.", EditorStyles.boldLabel);
            }
            DrawDefaultInspector();
        }

        public override bool RequiresConstantRepaint() {
            return true;
        }
    }
}

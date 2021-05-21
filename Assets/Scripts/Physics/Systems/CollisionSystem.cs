using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using BoxBoxConstraintManager = ConstraintManager<BoxBoxConstraintHelper, StandardConstraint>;
using ShadowEdgeConstraintManager = ConstraintManager<ShadowEdgeConstraintHelper, ShadowEdgeConstraint>;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(ShadowEdgeGenerationSystem))]
public class CollisionSystem : SystemBase {
    EntityQuery boxesQuery;
    EntityQuery hitShadBoxesQuery;
    EntityQuery lightSourcesQuery;
    // Warning: Friction no longer works when accumulateImpulses is false
    public static bool accumulateImpulses = true;
    public static bool warmStarting = true;
    public static bool positionCorrection = true;
    // In the future, this will be configurable on a per object basis
    public static float globalFriction = .5f;

    private BoxBoxConstraintManager boxBoxCM;
    private ShadowEdgeConstraintManager shadowEdgeCM;

    protected override void OnCreate() {
        boxesQuery = GetEntityQuery(typeof(Box));
        hitShadBoxesQuery = GetEntityQuery(typeof(Box), typeof(HitShadowsObject));
        lightSourcesQuery = GetEntityQuery(typeof(LightSource));

        boxBoxCM = new BoxBoxConstraintManager();
        shadowEdgeCM = new ShadowEdgeConstraintManager();
    }

    protected override void OnDestroy() {
        boxBoxCM.Dispose();
        shadowEdgeCM.Dispose();
    }

    protected override void OnUpdate() {
        NativeArray<Entity> boxEntities = boxesQuery.ToEntityArray(Allocator.TempJob);
        NativeArray<Entity> hitShadBoxEntities = hitShadBoxesQuery.ToEntityArray(Allocator.TempJob);
        NativeArray<Entity> lightSourceEntities = lightSourcesQuery.ToEntityArray(Allocator.TempJob);

        var boxes = GetComponentDataFromEntity<Box>(false);
        var velocities = GetComponentDataFromEntity<Velocity>(false);
        var lightSources = GetComponentDataFromEntity<LightSource>(false);

        float dt = Time.DeltaTime;

        boxBoxCM.helper.Update(
            boxes: boxes,
            boxVels: velocities,
            boxEntities: boxEntities,
            dt: dt
        );

        shadowEdgeCM.helper.Update(
            vels: velocities,
            boxes: boxes,
            lightSources: lightSources,
            shadowEdgeManifolds: World.GetOrCreateSystem<ShadowEdgeGenerationSystem>().GetShadowEdgeManifolds(),
            dt: dt
        );

        boxBoxCM.FindConstraints();
        shadowEdgeCM.FindConstraints();

        boxBoxCM.PreSteps(dt);
        shadowEdgeCM.PreSteps(dt);

        for (int i = 0; i < 10; i++) {
            boxBoxCM.ApplyImpulses(dt);
            shadowEdgeCM.ApplyImpulses(dt);
        }

        boxBoxCM.PostSteps();
        shadowEdgeCM.PostSteps();


        boxEntities.Dispose();
        hitShadBoxEntities.Dispose();
        lightSourceEntities.Dispose();
    }

    public struct DebugContactInfo {
        public float2 normal;
        public float2 contact;
        public int id;
    }

    public IEnumerable<DebugContactInfo> GetContactsForDebug() {
        foreach (var item in boxBoxCM.GetContactsForDebug()) {
            yield return item;
        }
        foreach (var item in shadowEdgeCM.GetContactsForDebug()) {
            yield return item;
        }
    }
}

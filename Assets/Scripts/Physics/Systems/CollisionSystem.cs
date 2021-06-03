using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using BoxBoxConstraintManager = ConstraintManager<BoxBoxConstraintHelper, StandardConstraint>;
using ShadowEdgeConstraintManager = ConstraintManager<ShadowEdgeConstraintHelper, ShadowEdgeConstraint>;
using ShadowCornerConstraintManager = ConstraintManager<ShadowCornerConstraintHelper, ShadowCornerConstraint>;

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
    private ShadowCornerConstraintManager shadowCornerCM;

    protected override void OnCreate() {
        boxesQuery = GetEntityQuery(typeof(Box));
        hitShadBoxesQuery = GetEntityQuery(typeof(Box), typeof(HitShadowsObject));
        lightSourcesQuery = GetEntityQuery(typeof(LightSource));

        boxBoxCM = new BoxBoxConstraintManager();
        shadowEdgeCM = new ShadowEdgeConstraintManager();
        shadowCornerCM = new ShadowCornerConstraintManager();
    }

    protected override void OnDestroy() {
        boxBoxCM.Dispose();
        shadowEdgeCM.Dispose();
        shadowCornerCM.Dispose();
    }

    protected override void OnUpdate() {
        // Don't start doing stuff until the second frame. This is useful
        // because when you start the game paused, the first frame is always
        // executed before the pause occurs.
        if (Time.ElapsedTime == 0) {
            return;
        }
        NativeArray<Entity> boxEntities = boxesQuery.ToEntityArray(Allocator.TempJob);
        NativeArray<Entity> hitShadBoxEntities = hitShadBoxesQuery.ToEntityArray(Allocator.TempJob);
        NativeArray<Entity> lightSourceEntities = lightSourcesQuery.ToEntityArray(Allocator.TempJob);

        var boxes = GetComponentDataFromEntity<Box>(false);
        var velocities = GetComponentDataFromEntity<Velocity>(false);
        var masses = GetComponentDataFromEntity<Mass>(false);
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
            partialConstraints: World.GetOrCreateSystem<ShadowEdgeGenerationSystem>().GetPartialEdgeConstraints(),
            masses: masses,
            dt: dt
        );

        shadowCornerCM.helper.Update(
            vels: velocities,
            boxes: boxes,
            lightSources: lightSources,
            partialConstraints: World.GetOrCreateSystem<ShadowEdgeGenerationSystem>().GetPartialCornerConstraints(),
            masses: masses,
            dt: dt
        );

        boxBoxCM.FindConstraints();
        shadowEdgeCM.FindConstraints();
        shadowCornerCM.FindConstraints();

        boxBoxCM.PreSteps(dt);
        shadowEdgeCM.PreSteps(dt);
        shadowCornerCM.PreSteps(dt);

        for (int i = 0; i < 10; i++) {
            boxBoxCM.ApplyImpulses(dt);
            shadowEdgeCM.ApplyImpulses(dt);
            shadowCornerCM.ApplyImpulses(dt);
        }

        boxBoxCM.PostSteps();
        shadowEdgeCM.PostSteps();
        shadowCornerCM.PostSteps();


        boxEntities.Dispose();
        hitShadBoxEntities.Dispose();
        lightSourceEntities.Dispose();
    }

    public struct DebugContactInfo {
        public float2 normal;
        public float2 contact;
        public int id;
    }

    // TODO: Make debug contact rendering not be broken
    //public IEnumerable<DebugContactInfo> GetContactsForDebug() {
    //    foreach (var item in boxBoxCM.GetContactsForDebug()) {
    //        yield return item;
    //    }
    //    foreach (var item in shadowEdgeCM.GetContactsForDebug()) {
    //        yield return item;
    //    }
    //}
}

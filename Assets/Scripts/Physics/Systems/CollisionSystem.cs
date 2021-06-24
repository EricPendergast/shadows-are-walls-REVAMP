using System.Collections.Generic;

using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;

using TwoWayPenFricCM = ConstraintManager<TwoWayPenFricConstraint, LambdaNT>;
using TwoWayPenCM = ConstraintManager<TwoWayPenConstraint, float>;
using ThreeWayPenCM = ConstraintManager<ThreeWayPenConstraint, float>;
using TwoWayTwoDOFCM = ConstraintManager<TwoWayTwoDOFConstraint, Unity.Mathematics.float2>;
using OneWayOneDOFCM = ConstraintManager<OneWayOneDOFConstraint, float>;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(ConstraintGenerationSystemGroup))]
public class CollisionSystem : SystemBase {
    EntityQuery boxesQuery;
    EntityQuery hitShadBoxesQuery;
    EntityQuery lightSourcesQuery;
    // Warning: Friction no longer works when accumulateImpulses is false
    public const bool accumulateImpulses = true;
    public const bool warmStarting = true;
    public const bool positionCorrection = true;
    // In the future, this will be configurable on a per object basis
    public const float globalFriction = .7f;
    public const float globalSoftness = 0;

    private float dt = 0;

    private TwoWayPenFricCM twoWayPenFricCM;
    private TwoWayPenCM twoWayPenCM;
    private ThreeWayPenCM threeWayPenCM;
    private TwoWayTwoDOFCM twoWayTwoDOFCM;
    private OneWayOneDOFCM oneWayOneDOFCM;

    public IEnumerable<IConstraintManager> DebugIterConstraintManagers() {
        yield return twoWayPenFricCM;
        yield return twoWayPenCM;
        yield return threeWayPenCM;
        yield return twoWayTwoDOFCM;
        yield return oneWayOneDOFCM;
    }

    protected override void OnCreate() {
        boxesQuery = GetEntityQuery(typeof(Box));
        hitShadBoxesQuery = GetEntityQuery(typeof(Box), typeof(HitShadowsObject));
        lightSourcesQuery = GetEntityQuery(typeof(LightSource));

        twoWayPenFricCM = new TwoWayPenFricCM();
        twoWayPenCM = new TwoWayPenCM();
        threeWayPenCM = new ThreeWayPenCM();
        twoWayTwoDOFCM = new TwoWayTwoDOFCM();
        oneWayOneDOFCM = new OneWayOneDOFCM();
    }

    protected override void OnDestroy() {
        twoWayPenFricCM.Dispose();
        twoWayPenCM.Dispose();
        threeWayPenCM.Dispose();
        twoWayTwoDOFCM.Dispose();
        oneWayOneDOFCM.Dispose();
    }

    protected override void OnUpdate() {
        var vels = GetComponentDataFromEntity<Velocity>(false);

        dt = Time.DeltaTime;

        var constraintGatherer = World.GetExistingSystem<ConstraintGatherSystem>();

        constraintGatherer.GiveConstraintsTo(ref twoWayPenFricCM);
        constraintGatherer.GiveConstraintsTo(ref twoWayPenCM);
        constraintGatherer.GiveConstraintsTo(ref threeWayPenCM);
        constraintGatherer.GiveConstraintsTo(ref twoWayTwoDOFCM);
        constraintGatherer.GiveConstraintsTo(ref oneWayOneDOFCM);
        
        twoWayPenFricCM.PreSteps(dt, ref vels);
        twoWayPenCM.PreSteps(dt, ref vels);
        threeWayPenCM.PreSteps(dt, ref vels);
        twoWayTwoDOFCM.PreSteps(dt, ref vels);
        oneWayOneDOFCM.PreSteps(dt, ref vels);

        // Do a dry run on the first frame. This is useful because when you
        // start the game paused, the first frame is always executed before the
        // pause occurs.
        if (Time.ElapsedTime != 0) {
            for (int i = 0; i < 10; i++) {
                twoWayPenFricCM.ApplyImpulses(dt);
                twoWayPenCM.ApplyImpulses(dt);
                threeWayPenCM.ApplyImpulses(dt);
                twoWayTwoDOFCM.ApplyImpulses(dt);
                oneWayOneDOFCM.ApplyImpulses(dt);
            }
        }

        twoWayPenFricCM.SaveWarmStartParameters();
        twoWayPenCM.SaveWarmStartParameters();
        threeWayPenCM.SaveWarmStartParameters();
        twoWayTwoDOFCM.SaveWarmStartParameters();
        oneWayOneDOFCM.SaveWarmStartParameters();
    }

    public struct DebugContactInfo {
        public float2 normal;
        public float2 contact;
        public int id;
    }

    public void DebugApplyWarmStart(IConstraint c, ComponentDataFromEntity<Velocity> vels, float dt) {
        foreach (var cm in DebugIterConstraintManagers()) {
            if (cm.DebugTryWarmStart(c, vels, dt)) {
                return;
            }
        }

        UnityEngine.Debug.LogError("Failed to warm start a constraint");
    }
}

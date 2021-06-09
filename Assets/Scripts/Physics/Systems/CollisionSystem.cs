using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;

using TwoWayPenFricCM = ConstraintManager<TwoWayPenFricConstraint, LambdaNT>;
using TwoWayPenCM = ConstraintManager<TwoWayPenConstraint, float>;
using ThreeWayPenCM = ConstraintManager<ThreeWayPenConstraint, float>;
using TwoWayTwoDOFCM = ConstraintManager<TwoWayTwoDOFConstraint, Unity.Mathematics.float2>;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(ConstraintGenerationSystemGroup))]
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
    public static float globalSoftness = 0;

    private TwoWayPenFricCM twoWayPenFricCM;
    private TwoWayPenCM twoWayPenCM;
    private ThreeWayPenCM threeWayPenCM;
    private TwoWayTwoDOFCM twoWayTwoDOFCM;

    protected override void OnCreate() {
        boxesQuery = GetEntityQuery(typeof(Box));
        hitShadBoxesQuery = GetEntityQuery(typeof(Box), typeof(HitShadowsObject));
        lightSourcesQuery = GetEntityQuery(typeof(LightSource));

        twoWayPenFricCM = new TwoWayPenFricCM();
        twoWayPenCM = new TwoWayPenCM();
        threeWayPenCM = new ThreeWayPenCM();
        twoWayTwoDOFCM = new TwoWayTwoDOFCM();
    }

    protected override void OnDestroy() {
        twoWayPenFricCM.Dispose();
        twoWayPenCM.Dispose();
        threeWayPenCM.Dispose();
        twoWayTwoDOFCM.Dispose();
    }

    protected override void OnUpdate() {

        var vels = GetComponentDataFromEntity<Velocity>(false);

        float dt = Time.DeltaTime;
        
        twoWayPenFricCM.PreSteps(dt, ref vels);
        twoWayPenCM.PreSteps(dt, ref vels);
        threeWayPenCM.PreSteps(dt, ref vels);
        twoWayTwoDOFCM.PreSteps(dt, ref vels);


        // Do a dry run on the first frame. This is useful because when you
        // start the game paused, the first frame is always executed before the
        // pause occurs.
        if (Time.ElapsedTime != 0) {
            for (int i = 0; i < 10; i++) {
                twoWayPenFricCM.ApplyImpulses(dt);
                twoWayPenCM.ApplyImpulses(dt);
                threeWayPenCM.ApplyImpulses(dt);
                twoWayTwoDOFCM.ApplyImpulses(dt);
            }
        }

        twoWayPenFricCM.PostSteps();
        twoWayPenCM.PostSteps();
        threeWayPenCM.PostSteps();
        twoWayTwoDOFCM.PostSteps();
    }

    public NativeList<TwoWayPenFricConstraint> GetStandardConstraintsInput() {
        return twoWayPenFricCM.GetConstraintsInput();
    }

    public NativeList<TwoWayPenConstraint> GetTwoWayPenConstraintsInput() {
        return twoWayPenCM.GetConstraintsInput();
    }

    public NativeList<ThreeWayPenConstraint> GetThreeWayPenConstraintsInput() {
        return threeWayPenCM.GetConstraintsInput();
    }

    public NativeList<TwoWayTwoDOFConstraint> GetTwoWayTwoDOFConstraintsInput() {
        return twoWayTwoDOFCM.GetConstraintsInput();
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

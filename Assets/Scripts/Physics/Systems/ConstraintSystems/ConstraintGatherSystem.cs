using System.Collections.Generic;

using Unity.Entities;
using Unity.Collections;

using TwoWayPenFricCM = ConstraintManager<TwoWayPenFricConstraint, LambdaNT>;
using TwoWayPenCM = ConstraintManager<TwoWayPenConstraint, float>;
using ThreeWayPenCM = ConstraintManager<ThreeWayPenConstraint, float>;
using TwoWayTwoDOFCM = ConstraintManager<TwoWayTwoDOFConstraint, Unity.Mathematics.float2>;
using OneWayOneDOFCM = ConstraintManager<OneWayOneDOFConstraint, float>;


[AlwaysUpdateSystem]
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateBefore(typeof(ConstraintGenerationSystemGroup))]
public class ConstraintGatherSystem : SystemBase {
    NativeList<TwoWayPenFricConstraint> constraints1;
    NativeList<TwoWayPenConstraint> constraints2;
    NativeList<ThreeWayPenConstraint> constraints3;
    NativeList<TwoWayTwoDOFConstraint> constraints4;
    NativeList<OneWayOneDOFConstraint> constraints5;

    protected override void OnCreate() {
        constraints1 = new NativeList<TwoWayPenFricConstraint>(Allocator.Persistent);
        constraints2 = new NativeList<TwoWayPenConstraint>(Allocator.Persistent);
        constraints3 = new NativeList<ThreeWayPenConstraint>(Allocator.Persistent);
        constraints4 = new NativeList<TwoWayTwoDOFConstraint>(Allocator.Persistent);
        constraints5 = new NativeList<OneWayOneDOFConstraint>(Allocator.Persistent);
    }

    protected override void OnDestroy() {
        constraints1.Dispose();
        constraints2.Dispose();
        constraints3.Dispose();
        constraints4.Dispose();
        constraints5.Dispose();
    }

    protected override void OnUpdate() {
        ClearConstraintBuffers();
    }

    public void ClearConstraintBuffers() {
        constraints1.Clear();
        constraints2.Clear();
        constraints3.Clear();
        constraints4.Clear();
        constraints5.Clear();
    }

    public void GiveConstraintsTo(ref TwoWayPenFricCM cm) {
        cm.RecieveConstraints(new NativeSlice<TwoWayPenFricConstraint>(constraints1));
    }
    public void GiveConstraintsTo(ref TwoWayPenCM cm) {
        cm.RecieveConstraints(new NativeSlice<TwoWayPenConstraint>(constraints2));
    }
    public void GiveConstraintsTo(ref ThreeWayPenCM cm) {
        cm.RecieveConstraints(new NativeSlice<ThreeWayPenConstraint>(constraints3));
    }
    public void GiveConstraintsTo(ref TwoWayTwoDOFCM cm) {
        cm.RecieveConstraints(new NativeSlice<TwoWayTwoDOFConstraint>(constraints4));
    }
    public void GiveConstraintsTo(ref OneWayOneDOFCM cm) {
        cm.RecieveConstraints(new NativeSlice<OneWayOneDOFConstraint>(constraints5));
    }

    public NativeList<TwoWayPenFricConstraint> GetTwoWayPenFricConstraintsInput() {
        return constraints1;
    }
    public NativeList<TwoWayPenConstraint> GetTwoWayPenConstraintsInput() {
        return constraints2;
    }
    public NativeList<ThreeWayPenConstraint> GetThreeWayPenConstraintsInput() {
        return constraints3;
    }
    public NativeList<TwoWayTwoDOFConstraint> GetTwoWayTwoDOFConstraintsInput() {
        return constraints4;
    }
    public NativeList<OneWayOneDOFConstraint> GetOneWayOneDOFConstraintsInput() {
        return constraints5;
    }

    public IEnumerable<IConstraint> DebugIterAllConstraints() {
        foreach (var c in constraints1) {
            yield return c;
        }
        foreach (var c in constraints2) {
            yield return c;
        }
        foreach (var c in constraints3) {
            yield return c;
        }
        foreach (var c in constraints4) {
            yield return c;
        }
        foreach (var c in constraints5) {
            yield return c;
        }
    }
}

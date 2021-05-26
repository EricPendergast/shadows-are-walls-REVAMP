using Unity.Entities;
using Unity.Collections;


// The corresponding constraing manager for this struct looks like this:
//using ShadowCornerConstraintManager = ConstraintManager<ShadowCornerConstraintHelper, ShadowCornerConstraint>;

public struct ShadowCornerConstraintHelper : ConstraintManagerHelper<ShadowCornerConstraint> {
    private ComponentDataFromEntity<Velocity> vels;
    private ComponentDataFromEntity<Mass> masses;
    private ComponentDataFromEntity<Box> boxes;
    private ComponentDataFromEntity<LightSource> lightSources;
    private NativeArray<ShadowCornerConstraint.Partial> partialConstraints;
    private float dt;

    public void Update(
            ComponentDataFromEntity<Velocity> vels,
            ComponentDataFromEntity<Mass> masses,
            ComponentDataFromEntity<Box> boxes,
            ComponentDataFromEntity<LightSource> lightSources,
            NativeArray<ShadowCornerConstraint.Partial> partialConstraints,
            float dt) {
        this.vels = vels;
        this.masses = masses;
        this.boxes = boxes;
        this.lightSources = lightSources;
        this.partialConstraints = partialConstraints;
        this.dt = dt;
    }

    public void ApplyImpulse(ref ShadowCornerConstraint constraint, float dt) {
        var v1 = vels[constraint.e1];
        var v2 = vels[constraint.e2];
        var v3 = vels[constraint.e3];

        constraint.ApplyImpulse(ref v1, ref v2, ref v3, dt);

        vels[constraint.e1] = v1;
        vels[constraint.e2] = v2;
        vels[constraint.e3] = v3;
    }

    public void PreStep(ref ShadowCornerConstraint constraint, float dt, Lambdas lambdas) {
        var v1 = vels[constraint.e1];
        var v2 = vels[constraint.e2];
        var v3 = vels[constraint.e3];

        constraint.PreStep(ref v1, ref v2, ref v3, dt, lambdas);

        vels[constraint.e1] = v1;
        vels[constraint.e2] = v2;
        vels[constraint.e3] = v3;
    }

    public void FillWithConstraints(NativeList<ShadowCornerConstraint> constraints) {
        foreach (var pConstraint in partialConstraints) {
            constraints.Add(new ShadowCornerConstraint(in pConstraint, masses, dt));
        }
    }
}

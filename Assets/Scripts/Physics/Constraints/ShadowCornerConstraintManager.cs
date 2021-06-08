using Unity.Entities;
using Unity.Collections;


// The corresponding constraint manager for this struct looks like this:
//using ShadowCornerConstraintManager = ConstraintManager<ShadowCornerConstraintHelper, ShadowCornerConstraint>;

public struct ShadowCornerConstraintHelper : ConstraintManagerHelper<ShadowCornerConstraint> {
    private ComponentDataFromEntity<Velocity> vels;

    public void Update(ComponentDataFromEntity<Velocity> vels) {
        this.vels = vels;
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
}

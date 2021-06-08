using Unity.Entities;

public struct ShadowEdgeConstraintHelper : ConstraintManagerHelper<ShadowEdgeConstraint> {
    private ComponentDataFromEntity<Velocity> vels;

    public void Update(ComponentDataFromEntity<Velocity> vels) {
        this.vels = vels;
    }

    public void ApplyImpulse(ref ShadowEdgeConstraint constraint, float dt) {
        var v1 = vels[constraint.e1];
        var v2 = vels[constraint.e2];

        constraint.ApplyImpulse(ref v1, ref v2, dt);

        vels[constraint.e1] = v1;
        vels[constraint.e2] = v2;
    }

    public void PreStep(ref ShadowEdgeConstraint constraint, float dt, Lambdas lambdas) {
        var v1 = vels[constraint.e1];
        var v2 = vels[constraint.e2];

        constraint.PreStep(ref v1, ref v2, dt, lambdas);

        vels[constraint.e1] = v1;
        vels[constraint.e2] = v2;
    }
}

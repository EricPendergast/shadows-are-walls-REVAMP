using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

[UpdateInGroup(typeof(ContactGenerationGroup))]
public class LightTrackingJointSystem : SystemBase {

    protected override void OnUpdate() {
        var output = World.GetOrCreateSystem<ConstraintGatherSystem>().GetOneWayOneDOFConstraintsInput();
        var masses = GetComponentDataFromEntity<Mass>();
        var positions = GetComponentDataFromEntity<Position>();

        float dt = Time.DeltaTime;

        Entities
        .WithReadOnly(masses)
        .WithReadOnly(positions)
        .ForEach((Entity e, in LightTrackingJoint joint, in Position lightPos) => {
            float targetAngle = Ang.SignedAngleOf(positions[joint.toTrack].pos - lightPos.pos);

            float targetOffset = Ang.SignedDistance(angleFrom: lightPos.rot, angleTo: targetAngle);

            var manifold = new TargetAngularVelocityManifold {
                e = e,
                id = e.GetHashCode() ^ 291019149,
                softness = joint.trackSoftness,
                targetAngVel = TargetAngularVelocityManifold.AngVelForTargetOffset(
                    offset: targetOffset,
                    maxSpeed: joint.trackSpeed,
                    dt: dt
                )
            };

            output.Add(new OneWayOneDOFConstraint(manifold, masses, dt));

        }).Schedule();
    }
}

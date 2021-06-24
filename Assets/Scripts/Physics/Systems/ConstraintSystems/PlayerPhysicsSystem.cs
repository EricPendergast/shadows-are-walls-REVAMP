using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;

using Physics.Math;

[UpdateInGroup(typeof(ConstraintGenerationSystemGroup), OrderFirst=true)]
public class PlayerFrictionSystem : SystemBase {

    protected override void OnUpdate() {
        var em = World.EntityManager;
        Entities
        .WithStructuralChanges()
        .WithAll<PlayerComponent, Friction>()
        .ForEach((Entity e, in PlayerComponent pc, in Friction f) => {
            em.RemoveComponent<Friction>(e);
        }).Run();
    }
}

[UpdateInGroup(typeof(ConstraintGenerationSystemGroup), OrderLast=true)]
public class PlayerPhysicsSystem : SystemBase {

    protected override void OnCreate() {
        RequireSingletonForUpdate<PlayerSettings>();
    }

    protected override void OnUpdate() {

        var input = World.GetOrCreateSystem<ConstraintGatherSystem>()
            .GetOneWayOneDOFConstraintsInput();
        var input2 = World.GetOrCreateSystem<ConstraintGatherSystem>()
            .GetTwoWayPenConstraintsInput();
        var input3 = World.GetOrCreateSystem<ConstraintGatherSystem>()
            .GetTwoWayOneDOFConstraintsInput();
        var masses = GetComponentDataFromEntity<Mass>();
        var positions = GetComponentDataFromEntity<Position>();
        var dt = Time.DeltaTime;

        var settings = GetSingleton<PlayerSettings>();

        Entities.ForEach((Entity e, in PlayerComponent pc, in Box playerBox, in Position playerPos, in DynamicBuffer<DirectContactStore> contacts) => {

            bool canJump = false;
            foreach (var contact in contacts) {
                if (math.dot(contact.normal, new float2(0, 1)) > settings.minJumpDotProd) {
                    canJump = true;

                    int uniqueHash = e.GetHashCode() ^ contact.other.GetHashCode() ^ contact.point.GetHashCode();

                    float2 r1 = 0;
                    float2 r2 = contact.point - positions[contact.other].pos;

                    var moveDir = Lin.Cross(contact.normal, 1);
                    if (math.dot(moveDir, new float2(pc.moveDirection)) < 0) {
                        moveDir *= -1;
                    }

                    if (pc.moveDirection != 0) {
                        var manifold = new MinRelativeVelocityManifold {
                            e1 = e,
                            e2 = contact.other,
                            id = uniqueHash ^ 347979364, // This id doesn't need to be consistent across frames.
                            minSpeedE1AlongNormal = settings.groundMoveSpeed,
                            softness = settings.groundMoveSoftness,
                            normal = moveDir,
                            r1 = r1,
                            r2 = r2
                        };

                        input2.Add(new TwoWayPenConstraint(manifold, masses, dt));
                    } else {
                        var manifold = new RelativeVelocityManifold {
                            e1 = e,
                            e2 = contact.other,
                            id = uniqueHash ^ 207704982, // This id doesn't need to be consistent across frames.
                            speedE1AlongNormal = 0,
                            softness = settings.groundMoveSoftness,
                            normal = moveDir,
                            r1 = r1,
                            r2 = r2
                        };
                        input3.Add(new TwoWayOneDOFConstraint(manifold, masses, dt));
                    }

                    if (pc.jumpPressed) {
                        var manifold = new MinRelativeVelocityManifold {
                            e1 = e,
                            e2 = contact.other,
                            id = uniqueHash ^ 416936944, // This id doesn't need to be consistent across frames.
                            minSpeedE1AlongNormal = settings.jumpSpeed,
                            softness = settings.jumpSoftness,
                            normal = new float2(0, 1),
                            r1 = r1,
                            r2 = r2
                        };

                        input2.Add(new TwoWayPenConstraint(manifold, masses, dt));
                    }
                }
            }

            if (!canJump && pc.moveDirection != 0) {
                var manifold = new TargetVelocityManifold {
                    softness = settings.airMoveSoftness,
                    id = e.GetHashCode() ^ 422603802,
                    r = float2.zero,
                    e = e,
                    normal = new float2(pc.moveDirection, 0),
                    targetSpeed = settings.airMoveSpeed
                };
                input.Add(new OneWayOneDOFConstraint(manifold, masses, dt));
            }

        }).Run();
    }
}

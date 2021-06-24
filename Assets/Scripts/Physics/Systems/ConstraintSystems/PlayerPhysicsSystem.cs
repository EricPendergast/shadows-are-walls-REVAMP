using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;

using Physics.Math;

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

                    if (pc.moveDirection != 0) {
                        var moveDir = Lin.Cross(contact.normal, 1);
                        if (math.dot(moveDir, new float2(pc.moveDirection)) < 0) {
                            moveDir *= -1;
                        }
                        var manifold = new RelativeVelocityManifold {
                            e1 = e,
                            e2 = contact.other,
                            id = uniqueHash ^ 347979364, // This id doesn't need to be consistent across frames.
                            minSpeedE1AlongNormal = settings.groundMoveSpeed,
                            softness = settings.groundMoveSoftness,
                            normal = moveDir,
                            r1 = 0,//contact.point - positions[e].pos,
                            r2 = contact.point - positions[contact.other].pos,
                        };

                        input2.Add(new TwoWayPenConstraint(manifold, masses, dt));
                    } else {

                    }

                    if (pc.jumpPressed) {
                        var manifold = new RelativeVelocityManifold {
                            e1 = e,
                            e2 = contact.other,
                            id = uniqueHash ^ 416936944, // This id doesn't need to be consistent across frames.
                            minSpeedE1AlongNormal = settings.jumpSpeed,
                            softness = settings.jumpSoftness,
                            normal = new float2(0, 1),
                            r1 = float2.zero,
                            r2 = contact.point - positions[contact.other].pos,
                        };

                        input2.Add(new TwoWayPenConstraint(manifold, masses, dt));
                    }
                }
            }

            if (!canJump && pc.moveDirection != 0) {
                var manifold = new TargetVelocityManifold {
                    softness = settings.airMoveSoftness,
                    id = e.GetHashCode() ^ 347979364,
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

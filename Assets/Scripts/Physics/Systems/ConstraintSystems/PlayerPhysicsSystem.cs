using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;

using Physics.Math;

[UpdateInGroup(typeof(PreContactGenerationGroup))]
public class PlayerNearbyDetectorSystem : SystemBase {
    private struct SetPosition {
        public Entity e;
        public Position p;
    }
    private struct SetBox {
        public Entity e;
        public Box b;
    }
    protected override void OnUpdate() {

        var settings = GetSingleton<PlayerSettings>();

        var positionChanges = new NativeList<SetPosition>(Allocator.TempJob);
        var boxChanges = new NativeList<SetBox>(Allocator.TempJob);

        // make the player's swappable detector follow the player position
        Entities.WithAll<ActivePlayer>().ForEach((in PlayerRoot playerRoot, in Position pos, in Box box) => {
            
            positionChanges.Add(new SetPosition{e=playerRoot.swappableDetector, p = pos});
            boxChanges.Add(
                new SetBox{e=playerRoot.swappableDetector, b=new Box {
                    height = box.height + settings.swapDetectorBorder*2,
                    width = box.width + settings.swapDetectorBorder*2,
                    id = playerRoot.swappableDetector.GetHashCode()
                }
            });
        }).Schedule();

        var positions = GetComponentDataFromEntity<Position>(isReadOnly: false);
        var boxes = GetComponentDataFromEntity<Box>(isReadOnly: false);

        Job.WithCode(() => {
            foreach (var pos in positionChanges) {
                positions[pos.e] = pos.p;
            }
            foreach (var box in boxChanges) {
                boxes[box.e] = box.b;
            }
        }).Schedule();

        positionChanges.Dispose(Dependency);
        boxChanges.Dispose(Dependency);
    }
}

// This system ensures that the active player has friction disabled
[UpdateInGroup(typeof(PreContactGenerationGroup))]
public class PlayerFrictionSystem : SystemBase {
    private struct PreviousPlayerFriction : IComponentData {
        public Friction prevFriction;
    }

    protected override void OnUpdate() {
        var ecbSystem = World.GetOrCreateSystem<EndFixedStepSimulationEntityCommandBufferSystem>();
        var ecb = ecbSystem.CreateCommandBuffer().AsParallelWriter();

        Entities
            .WithAll<ActivePlayer>()
            .WithNone<PreviousPlayerFriction>()
            .ForEach((int entityInQueryIndex, Entity e, in Friction friction) => {
                ecb.AddComponent(entityInQueryIndex, e, new PreviousPlayerFriction {
                    prevFriction = friction
                });
                ecb.RemoveComponent<Friction>(entityInQueryIndex, e);
            }).ScheduleParallel();

        Entities
            .WithNone<ActivePlayer, Friction>()
            .ForEach((int entityInQueryIndex, Entity e, in PreviousPlayerFriction prevFriction) => {
                ecb.AddComponent<Friction>(entityInQueryIndex, e,
                    prevFriction.prevFriction
                );
                ecb.RemoveComponent<PreviousPlayerFriction>(entityInQueryIndex, e);
            }).ScheduleParallel();

        Entities
            .WithNone<ActivePlayer>()
            .WithAll<Friction, PreviousPlayerFriction>()
            .ForEach((int entityInQueryIndex, Entity e) => {
                ecb.RemoveComponent<PreviousPlayerFriction>(entityInQueryIndex, e);
            }).ScheduleParallel();

        ecbSystem.AddJobHandleForProducer(Dependency);
    }
}

[UpdateInGroup(typeof(PostContactGenerationGroup))]
public class PlayerPhysicsSystem : SystemBase {

    protected override void OnCreate() {
        RequireSingletonForUpdate<PlayerSettings>();
        RequireSingletonForUpdate<PlayerControlInputs>();
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
        var controls = GetSingleton<PlayerControlInputs>();

        Entities
            .WithAll<ActivePlayer>()
            .ForEach((Entity e, in Box playerBox, in Position playerPos, in DynamicBuffer<DirectContactStore> contacts) => {

            {
                float rotationTarget = math.round(playerPos.rot / (math.PI/2))*(math.PI/2);
                float rotationOffset = rotationTarget - playerPos.rot;
                float rotationOffsetSign = math.sign(rotationOffset);
                var manifold = new TargetAngularVelocityManifold {
                    e = e,
                    id = e.GetHashCode() ^ 286091097,
                    softness = settings.rotationCorrectionSoftness,
                    targetAngVel = math.min(rotationOffset/dt, rotationOffsetSign*settings.rotationCorrectionSpeed)
                };

                input.Add(new OneWayOneDOFConstraint(manifold, masses, dt));
            }


            bool canJump = false;
            foreach (var contact in contacts) {
                if (math.dot(contact.normal, new float2(0, 1)) > settings.minJumpDotProd) {
                    canJump = true;

                    int uniqueHash = e.GetHashCode() ^ contact.other.GetHashCode() ^ contact.point.GetHashCode();

                    float2 r1 = 0;
                    float2 r2 = contact.point - positions[contact.other].pos;

                    var moveDir = Lin.Cross(contact.normal, 1);
                    if (math.dot(moveDir, new float2(controls.moveDirection)) < 0) {
                        moveDir *= -1;
                    }

                    if (controls.moveDirection != 0) {
                        var manifold = new TargetVelocityManifold {
                            softness = settings.groundMoveSoftness,
                            id = uniqueHash ^ 127490702,
                            r = float2.zero,
                            e = e,
                            normal = new float2(controls.moveDirection, 0),
                            targetSpeed = settings.groundMoveSpeed
                        };
                        input.Add(new OneWayOneDOFConstraint(manifold, masses, dt));
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

                    if (controls.jumpPressed) {
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

            if (!canJump && controls.moveDirection != 0) {
                var manifold = new TargetVelocityManifold {
                    softness = settings.airMoveSoftness,
                    id = e.GetHashCode() ^ 422603802,
                    r = float2.zero,
                    e = e,
                    normal = new float2(controls.moveDirection, 0),
                    targetSpeed = settings.airMoveSpeed
                };
                input.Add(new OneWayOneDOFConstraint(manifold, masses, dt));
            }

        }).Run();
    }
}

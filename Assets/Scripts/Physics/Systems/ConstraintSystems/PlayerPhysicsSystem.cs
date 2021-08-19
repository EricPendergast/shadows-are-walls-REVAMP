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

        // If ActivePlayer, set friction to zero and store its previous value in PreviousPlayerFriction
        Entities
            .WithAll<ActivePlayer>()
            .WithNone<PreviousPlayerFriction>()
            .WithAll<Friction>()
            .ForEach((int entityInQueryIndex, Entity e, in Friction friction) => {
                ecb.AddComponent(entityInQueryIndex, e, new PreviousPlayerFriction {
                    prevFriction = friction
                });
                ecb.SetComponent(entityInQueryIndex, e, new Friction{friction=0});
            }).ScheduleParallel();

        // If ActivePlayer is removed, revert back to the friction stored in PreviousPlayerFriction
        Entities
            .WithNone<ActivePlayer>()
            .WithAll<PreviousPlayerFriction>()
            .WithNone<Friction>()
            .ForEach((int entityInQueryIndex, Entity e, in PreviousPlayerFriction prevFriction) => {
                ecb.AddComponent<Friction>(entityInQueryIndex, e, prevFriction.prevFriction);
                ecb.RemoveComponent<PreviousPlayerFriction>(entityInQueryIndex, e);
            }).ScheduleParallel();

        Entities
            .WithNone<ActivePlayer>()
            .WithAll<PreviousPlayerFriction>()
            .WithAll<Friction>()
            .ForEach((int entityInQueryIndex, Entity e, in PreviousPlayerFriction prevFriction) => {
                ecb.SetComponent(entityInQueryIndex, e, prevFriction.prevFriction);
                ecb.RemoveComponent<PreviousPlayerFriction>(entityInQueryIndex, e);
            }).ScheduleParallel();

        var controls = GetSingleton<PlayerControlInputs>();

        // Restore player friction when not moving
        Entities
            .WithAll<ActivePlayer, Friction, PreviousPlayerFriction>()
            .ForEach((ref Friction fric, in PreviousPlayerFriction prevFric) => {
                if (controls.moveDirection == 0) {
                    fric = prevFric.prevFriction;
                } else {
                    fric.friction = 0;
                }
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
            .ForEach((Entity e, ref Velocity v, in Box playerBox, in Position playerPos, in DynamicBuffer<DirectContactStore> contacts, in DynamicBuffer<ShadowContactStore> shadowContacts) => {

            {// Adding a constraint to rotate the player to be axis aligned
                float rotationTarget = math.round(playerPos.rot / (math.PI/2))*(math.PI/2);
                float rotationOffset = rotationTarget - playerPos.rot;
                var manifold = new TargetAngularVelocityManifold {
                    e = e,
                    id = e.GetHashCode() ^ 286091097,
                    softness = settings.rotationCorrectionSoftness,
                    targetAngVel = TargetAngularVelocityManifold.AngVelForTargetOffset(
                        offset: rotationOffset,
                        maxSpeed: settings.rotationCorrectionSpeed,
                        dt: dt
                    )
                };

                input.Add(new OneWayOneDOFConstraint(manifold, masses, dt));
            }

            bool canJump = false;
            void HandleContact(float2 point, float2 normal, Entity other) {
                if (math.dot(normal, new float2(0, 1)) > settings.minJumpDotProd) {
                    canJump = true;

                    int uniqueHash = e.GetHashCode() ^ other.GetHashCode() ^ point.GetHashCode();

                    float2 r1 = 0;
                    float2 r2 = point - positions[other].pos;

                    var moveDir = Lin.Cross(normal, 1);
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
                    }
                }
            }


            foreach (var contact in contacts) {
                HandleContact(point: contact.point, normal: contact.normal, other: contact.other);
            }
            foreach (var contact in shadowContacts) {
                HandleContact(point: contact.point, normal: contact.normal, other: contact.other);
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

            if (controls.jumpPressed && canJump) {
                var manifold = new TargetVelocityManifold {
                    e = e,
                    id = e.GetHashCode() ^ 404018171,
                    softness = settings.jumpSoftness,
                    normal = new float2(0, 1),
                    r = float2.zero,
                    targetSpeed = settings.jumpSpeed
                };
            
                input.Add(new OneWayOneDOFConstraint(manifold, masses, dt));
            }
        }).Run();
    }
}

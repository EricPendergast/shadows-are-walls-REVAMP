using System.Collections.Generic;
using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Burst;
using Unity.Jobs;

using UnityEngine;

using Physics.Math;

[UpdateInGroup(typeof(ConstraintGenerationSystemGroup))]
public class DirectConstraintSystem : SystemBase {

    private struct Emitter {
        public static List<IDebuggableConstraint> debuggableConstraints;
        public NativeList<TwoWayPenFricConstraint>? constraints;

        [BurstDiscard]
        public void EmitDebuggableConstraint(Geometry.Manifold m, TwoWayPenFricConstraint.Partial p, TwoWayPenFricConstraint c, bool useContact1, float dt) {
            if (debuggableConstraints != null) {
                debuggableConstraints.Add(new DebuggableConstraint(m, p, c, useContact1, dt: dt));
            }
        }

        public void EmitConstraint(TwoWayPenFricConstraint constraint) {
            if (constraints != null) {
                constraints.Value.Add(constraint);
            }
        }
    }

    EntityQuery boxesQuery;

    protected override void OnUpdate() {
        Emit(
            new Emitter{
                constraints = World.GetOrCreateSystem<ConstraintGatherSystem>().GetTwoWayPenFricConstraintsInput()
            },
            Time.DeltaTime,
            useBurst: true
        );
    }

    public IEnumerable<IDebuggableConstraint> GetDebuggableConstraints(float dt) {
        var ret = new List<IDebuggableConstraint>();
        Emitter.debuggableConstraints = ret;
        Emit(new Emitter(), dt, false);
        Emitter.debuggableConstraints = null;
        return ret;
    }

    private struct Env {
        public ComponentDataFromEntity<Box> boxes;
        public ComponentDataFromEntity<Mass> masses;
        public ComponentDataFromEntity<Position> positions;
        public NativeArray<Entity> boxEntities;
        public float dt;
    }

    private void Emit(Emitter emitter, float dt, bool useBurst) {
        NativeArray<Entity> boxEntities = boxesQuery.ToEntityArrayAsync(Allocator.TempJob, out JobHandle jh);
        Dependency = JobHandle.CombineDependencies(Dependency, jh);

        var env = new Env {
            boxes = GetComponentDataFromEntity<Box>(),
            masses = GetComponentDataFromEntity<Mass>(),
            positions = GetComponentDataFromEntity<Position>(),
            dt = dt,
            boxEntities = boxEntities
        };

        if (useBurst) {
            var boxes = env.boxes;
            var masses = env.masses;
            var positions = env.positions;
            Entities
            .WithReadOnly(boxes)
            .WithReadOnly(boxEntities)
            .WithReadOnly(masses)
            .WithReadOnly(positions)
            .WithStoreEntityQueryInField(ref boxesQuery)
            .ForEach((int entityInQueryIndex, in Box box) => {
                var env = new Env {
                    boxes = boxes,
                    boxEntities = boxEntities,
                    dt = dt,
                    masses = masses,
                    positions = positions
                };
                EmitForBox(entityInQueryIndex, in box, in emitter, in env);
            }).Schedule();
        } else {
            Entities
            .WithoutBurst()
            .ForEach((int entityInQueryIndex, in Box box) => {
                EmitForBox(entityInQueryIndex, in box, in emitter, in env);
            }).Run();
        }

        Dependency = boxEntities.Dispose(Dependency);
    }

    private static void EmitForBox(int boxIndex, in Box box, in Emitter emitter, in Env env) {
        for (int j = boxIndex+1; j < env.boxEntities.Length; j++ ) {
            Entity box1 = env.boxEntities[boxIndex];
            Entity box2 = env.boxEntities[j];

            var manifoldNullable = Geometry.GetIntersectData(
                env.boxes[box1].ToRect(env.positions[box1]),
                env.boxes[box2].ToRect(env.positions[box2])
            );

            if (manifoldNullable is Geometry.Manifold manifold) {
                AddConstraint(emitter, env, box1, box2, manifold, true);

                if (manifold.contact2 is Geometry.Contact contact) {

                    AddConstraint(emitter, env, box1, box2, manifold, false);
                    Debug.Assert(!manifold.contact1.id.Equals(contact.id), "Duplicate contact ids within the same manifold");
                }
            }
        }
    }

    private static void AddConstraint(in Emitter emitter, in Env env, Entity e1, Entity e2, Geometry.Manifold manifold, bool useContact1) {
        Box box1 = env.boxes[e1];
        Box box2 = env.boxes[e2];

        if (env.masses[e1].mass == math.INFINITY && env.masses[e2].mass == math.INFINITY) {
            return;
        }

        var partial = new TwoWayPenFricConstraint.Partial(
            e1, e2,
            env.positions[e1], env.positions[e2],
            manifold,
            useContact1
        );

        var c = new TwoWayPenFricConstraint(
            partial,
            env.masses,
            dt: env.dt,
            beta: CollisionSystem.positionCorrection ? .1f : 0,
            delta_slop: -.01f
        );

        emitter.EmitConstraint(c);
        emitter.EmitDebuggableConstraint(manifold, partial, c, useContact1, env.dt);
    }

    private struct DebuggableConstraint : IDebuggableConstraint {
        private Geometry.Manifold m;
        private IConstraint constraint;

        Float6 M_inv;
        float dt;
        bool useContact1;
        TwoWayPenFricConstraint.Partial p;

        public DebuggableConstraint(Geometry.Manifold m, TwoWayPenFricConstraint.Partial p, TwoWayPenFricConstraint c, bool useContact1, float dt) {
            this.m = m;
            this.useContact1 = useContact1;
            constraint = c;
            M_inv = c.M_inv;
            this.dt = dt;
            this.p = p;
        }

        public IDebuggableConstraint Clone() {
            var ret = this;
            ret.constraint = ret.constraint.Clone();
            return ret;
        }

        public void DrawGizmos(IDebuggableConstraint.DrawGizmosSettings settings) {
        
            Vector2 normal = m.normal;
            void Draw(Vector2 contact, int id) {
                id = math.abs(id);
            
                Gizmos.color = new Color(
                        (id % 4591 % 256)/256.0f, 
                        (id % 5347 % 256)/265.0f,
                        (id % 3797 % 256)/265.0f);

                Gizmos.DrawRay(contact, normal);

                Gizmos.DrawSphere(contact, .05f);
            }
            if (useContact1) {
                Draw(m.contact1.point, m.contact1.id);
            } else {
                Draw(m.contact2.Value.point, m.contact2.Value.id);
            }
        }

        public void SetConstants(IDebuggableConstraint.Constants constants) {
            constraint = new TwoWayPenFricConstraint(p, M_inv, dt: dt, beta: constants.beta, delta_slop: constants.delta_slop);
        }

        public IConstraint GetConstraint() {
            return constraint;
        }
    }
}

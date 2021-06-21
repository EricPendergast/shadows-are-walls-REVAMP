using System.Collections.Generic;
using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Burst;

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
    ComponentDataFromEntity<Box> boxes;
    ComponentDataFromEntity<Position> positions;
    ComponentDataFromEntity<Mass> masses;

    protected override void OnCreate() {
        boxesQuery = GetEntityQuery(typeof(Box));
    }

    protected override void OnUpdate() {
        Emit(
            new Emitter{
                constraints = World.GetOrCreateSystem<ConstraintGatherSystem>().GetTwoWayPenFricConstraintsInput()
            }, Time.DeltaTime
        );
    }

    public IEnumerable<IDebuggableConstraint> GetDebuggableConstraints(float dt) {
        var ret = new List<IDebuggableConstraint>();
        Emitter.debuggableConstraints = ret;
        Emit(new Emitter(), dt);
        Emitter.debuggableConstraints = null;
        return ret;
    }

    private void Emit(Emitter emitter, float dt) {
        NativeArray<Entity> boxEntities = boxesQuery.ToEntityArray(Allocator.TempJob);
        boxes = GetComponentDataFromEntity<Box>();
        masses = GetComponentDataFromEntity<Mass>();
        positions = GetComponentDataFromEntity<Position>();

        for (int i = 0; i < boxEntities.Length; i++ ) {
            for (int j = i+1; j < boxEntities.Length; j++ ) {
                Entity box1 = boxEntities[i];
                Entity box2 = boxEntities[j];

                var manifoldNullable = GetManifold(box1, box2);

                if (manifoldNullable is Geometry.Manifold manifold) {


                    AddConstraint(emitter, box1, box2, manifold, true, dt);

                    if (manifold.contact2 is Geometry.Contact contact) {

                        AddConstraint(emitter, box1, box2, manifold, false, dt);
                        Debug.Assert(!manifold.contact1.id.Equals(contact.id), "Duplicate contact ids within the same manifold");
                    }
                }
            }
        }

        boxEntities.Dispose();
    }

    private void AddConstraint(Emitter emitter, Entity e1, Entity e2, Geometry.Manifold manifold, bool useContact1, float dt) {
        Box box1 = boxes[e1];
        Box box2 = boxes[e2];

        if (masses[e1].mass == math.INFINITY && masses[e2].mass == math.INFINITY) {
            return;
        }

        var partial = new TwoWayPenFricConstraint.Partial(
            e1, e2,
            positions[e1], positions[e2],
            manifold,
            useContact1
        );

        var c = new TwoWayPenFricConstraint(
            partial,
            masses,
            dt: dt,
            beta: CollisionSystem.positionCorrection ? .1f : 0,
            delta_slop: -.01f
        );

        emitter.EmitConstraint(c);
        emitter.EmitDebuggableConstraint(manifold, partial, c, useContact1, dt);
    }

    private Geometry.Manifold? GetManifold(Entity box1, Entity box2) {
        return Geometry.GetIntersectData(
            boxes[box1].ToRect(positions[box1]),
            boxes[box2].ToRect(positions[box2])
        );
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

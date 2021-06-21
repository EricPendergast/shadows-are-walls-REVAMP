using System.Collections.Generic;

using Unity.Entities;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Burst;

using UnityEngine;

using RJConstraint = TwoWayTwoDOFConstraint;

[UpdateInGroup(typeof(ConstraintGenerationSystemGroup))]
public class RevoluteJointSystem : SystemBase {

    private struct Emitter {
        public static List<IDebuggableConstraint> debuggableConstraints;
        public NativeList<TwoWayTwoDOFConstraint>? constraints;

        [BurstDiscard]
        public void EmitDebuggableConstraint(RevoluteJointManifold m, RJConstraint c, ComponentDataFromEntity<Position> positions, float dt) {
            if (debuggableConstraints != null) {
                debuggableConstraints.Add(new DebuggableConstraint(m, c, positions, dt));
            }
        }

        public void EmitConstraint(TwoWayTwoDOFConstraint constraint) {
            if (constraints != null) {
                constraints.Value.Add(constraint);
            }
        }
    }

    protected override void OnUpdate() {
        var constraintsOut = World.GetOrCreateSystem<ConstraintGatherSystem>().  GetTwoWayTwoDOFConstraintsInput();
        Emit(
            new Emitter{
                constraints = constraintsOut
            },
            useBurst: true,
            Time.DeltaTime
        );
    }

    private void Emit(Emitter emitter, bool useBurst, float dt) {

        var masses = GetComponentDataFromEntity<Mass>();
        var positions = GetComponentDataFromEntity<Position>();

        if (useBurst) {
            Entities.
                ForEach((Entity jointEntity, in RevoluteJoint joint) => {
                var m = GetManifold(jointEntity, joint, positions);
                var c = new TwoWayTwoDOFConstraint(
                    m,
                    masses,
                    dt
                );
                emitter.EmitConstraint(c);
                emitter.EmitDebuggableConstraint(m, c, positions, dt);
            }).Run();
        } else {
            // This code is exactly the same as above except with a
            // WithoutBurst(). Not sure if there's a better solution
            // here.
            Entities
                .WithoutBurst()
                .ForEach((Entity jointEntity, in RevoluteJoint joint) => {
                var m = GetManifold(jointEntity, joint, positions);
                var c = new TwoWayTwoDOFConstraint(
                    m,
                    masses,
                    dt
                );
                emitter.EmitConstraint(c);
                emitter.EmitDebuggableConstraint(m, c, positions, dt);
            }).Run();
        }
    }

    private static RevoluteJointManifold GetManifold(Entity jointEntity, RevoluteJoint joint, ComponentDataFromEntity<Position> positions) {
        var pos1 = positions[joint.e1];
        var pos2 = positions[joint.e2];

        float2 x1 = pos1.pos;
        float2 x2 = pos2.pos;
        float2 r1 = pos1.LocalDirectionToGlobal(joint.localAnchor1);
        float2 r2 = pos2.LocalDirectionToGlobal(joint.localAnchor2);

        return new RevoluteJointManifold{
            e1 = joint.e1,
            e2 = joint.e2,
            r1 = r1,
            r2 = r2,
            delta = x2 + r2 - x1 - r1,
            id = jointEntity.GetHashCode() ^ 19378933,
            softness = joint.softness,
            beta = joint.beta
        };
    }

    public IEnumerable<IDebuggableConstraint> GetDebuggableConstraints(float dt) {
        var ret = new List<IDebuggableConstraint>();
        Emitter.debuggableConstraints = ret;
        Emit(new Emitter(), useBurst: false, dt);
        Emitter.debuggableConstraints = null;
        return ret;
    }

    private struct DebuggableConstraint : IDebuggableConstraint {
        private RevoluteJointManifold m;
        private IConstraint constraint;
        private Vector2 x1;
        private Vector2 x2;

        Float6 M_inv;
        float dt;

        public DebuggableConstraint(RevoluteJointManifold m, TwoWayTwoDOFConstraint c, ComponentDataFromEntity<Position> positions, float dt) {
            this.m = m;

            constraint = c;
            x1 = positions[m.e1].pos;
            x2 = positions[m.e2].pos;
            M_inv = c.M_inv;
            this.dt = dt;
        }

        public IDebuggableConstraint Clone() {
            var ret = this;
            ret.constraint = ret.constraint.Clone();
            return ret;
        }

        public void DrawGizmos(IDebuggableConstraint.DrawGizmosSettings settings) {
            Vector2 m1 = x1 + (Vector2)m.r1;
            Vector2 m2 = x2 + (Vector2)m.r2;
    
            Gizmos.color = settings.color;
            Gizmos.DrawLine(x1, m1);
            Gizmos.DrawSphere(m1, settings.drawRadius);
    
            Gizmos.color = settings.springStretchColor;
            Gizmos.DrawLine(m1, m2);
    
            Gizmos.color = settings.color;
            Gizmos.DrawLine(x2, m2);
            Gizmos.DrawSphere(m2, settings.drawRadius);
        }

        public void SetConstants(IDebuggableConstraint.Constants constants) {
            if (constants.beta is float beta) {
                m.beta = beta;
            }
            if (constants.softness is float softness) {
                m.softness = softness;
            }
            constraint = new TwoWayTwoDOFConstraint(m, M_inv, dt);
        }

        public IConstraint GetConstraint() {
            return constraint;
        }
    }
}

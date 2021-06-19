using System.Collections.Generic;

using Unity.Entities;
using Unity.Burst;
using Unity.Collections;

using UnityEngine;

public interface IConstraint {
    public void ApplyImpulses(ComponentDataFromEntity<Velocity> vels, float dt);
    public IConstraint Clone();
}

public interface IWarmStartConstraint<LambdaType> : IConstraint where LambdaType : struct {
    public int id {get;}
    public LambdaType GetAccumulatedLambda();
    public void WarmStart(ComponentDataFromEntity<Velocity> vels, float dt, LambdaType lambdaInit);
}

public interface IConstraintManager {
    void PreSteps(float dt, ref ComponentDataFromEntity<Velocity> vels);
    void ApplyImpulses(float dt);
    bool DebugTryWarmStart(IConstraint input, ComponentDataFromEntity<Velocity> vels, float dt);
}

public class ConstraintManager<ConstraintType, LambdaType> : IConstraintManager
        where ConstraintType : struct, IWarmStartConstraint<LambdaType>
        where LambdaType : struct {

    private NativeSlice<ConstraintType> constraints;
    private WarmStartManager warmStart;
    private ComponentDataFromEntity<Velocity> vels;

    public ConstraintManager() {
        warmStart = new WarmStartManager();
    }

    public void PreSteps(float dt, ref ComponentDataFromEntity<Velocity> vels) {
        this.vels = vels;
        for (int j = 0; j < constraints.Length; j++) {
            var c = constraints[j];

            c.WarmStart(vels, dt, warmStart.GetLambda(c.id));

            // TODO: Non readonly structs are EVIL
            constraints[j] = c;
        }
    }

    public void ApplyImpulses(float dt) {
        for (int j = 0; j < constraints.Length; j++) {
            var c = constraints[j];

            c.ApplyImpulses(vels, dt);

            // TODO: Non readonly structs are EVIL
            constraints[j] = c;
        }
    }

    public void SaveWarmStartParameters() {
        warmStart.SaveLambdas(constraints);
    }

    public void RecieveConstraints(NativeSlice<ConstraintType> constraints) {
        this.constraints = constraints;
    }

    // Passing in vels because this.vels may be invalidated
    [BurstDiscard]
    public bool DebugTryWarmStart(IConstraint c, ComponentDataFromEntity<Velocity> vels, float dt) {
        if (c is ConstraintType constraint) {
            constraint.WarmStart(vels, dt, warmStart.GetLambda(constraint.id));
            return true;
        }
        return false;
    }

    public void Dispose() {
        warmStart.Dispose();
    }

    public class WarmStartManager {
        // Maps from contact id to the accumulated lambda of that contact last
        // frame. Used for warm starting.
        private NativeHashMap<int, LambdaType> prevLambdas;

        public WarmStartManager() {
            prevLambdas = new NativeHashMap<int, LambdaType>(0, Allocator.Persistent);
        }

        public LambdaType GetLambda(int id) {
            LambdaType lambda;
            if (CollisionSystem.warmStarting && prevLambdas.TryGetValue(id, out var l)) {
                lambda = l;
            } else {
                lambda = new LambdaType();
            }
            return lambda;
        }

        public void SaveLambdas(NativeSlice<ConstraintType> constraints) {
            prevLambdas.Clear();
            foreach (var constraint in constraints) {
                Debug.Assert(!prevLambdas.ContainsKey(constraint.id), "Duplicate contact id: " + constraint.id.ToString());
                prevLambdas[constraint.id] = constraint.GetAccumulatedLambda();
            }
        }

        public void Dispose() {
            prevLambdas.Dispose();
        }
    }
}

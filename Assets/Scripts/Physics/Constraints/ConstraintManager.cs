using Unity.Entities;
using Unity.Collections;
using UnityEngine;

public interface IConstraint<LambdaType> where LambdaType : struct {
    public int id {get;}
    public LambdaType GetAccumulatedLambda();

    public void PreStep(ComponentDataFromEntity<Velocity> vels, float dt, LambdaType lambdaInit);
    public void ApplyImpulses(ComponentDataFromEntity<Velocity> vels, float dt);
}

public class ConstraintManager<ConstraintType, LambdaType> 
        where ConstraintType : struct, IConstraint<LambdaType>
        where LambdaType : struct {

    private NativeList<ConstraintType> constraints;
    private WarmStartManager warmStart;
    private ComponentDataFromEntity<Velocity> vels;

    public ConstraintManager() {
        constraints = new NativeList<ConstraintType>(0, Allocator.Persistent);
        warmStart = new WarmStartManager();
    }

    public void PreSteps(float dt, ref ComponentDataFromEntity<Velocity> vels) {
        this.vels = vels;
        for (int j = 0; j < constraints.Length; j++) {
            var c = constraints[j];

            c.PreStep(vels, dt, warmStart.GetLambda(c.id));

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

    public void PostSteps() {
        warmStart.SaveLambdas(constraints);
        constraints.Clear();
    }

    public NativeList<ConstraintType> GetConstraintsInput() {
        return constraints;
    }

    //public IEnumerable<CollisionSystem.DebugContactInfo> GetContactsForDebug() {
    //    foreach (var constraint in constraints) {
    //        yield return new CollisionSystem.DebugContactInfo{normal = constraint.normal, contact = constraint.contact, id = constraint.id};
    //    }
    //}

    public void Dispose() {
        warmStart.Dispose();
        constraints.Dispose();
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

        public void SaveLambdas(NativeArray<ConstraintType> constraints) {
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

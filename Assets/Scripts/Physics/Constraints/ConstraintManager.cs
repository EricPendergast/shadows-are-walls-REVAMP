using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using UnityEngine;

using Physics.Math;

using ContactId = Physics.Math.Geometry.ContactId;

public interface IConstraint {
    public int id {get;}
    //public float2 normal {get;}
    //public float2 contact {get;}
    public Lambdas GetAccumulatedLambdas();
}

public interface ConstraintManagerHelper<C> where C : struct, IConstraint {
     void ApplyImpulse(ref C constraint, float dt);
     void PreStep(ref C constraint, float dt, Lambdas lambdas);
     void FillWithConstraints(NativeList<C> constraints);
}

public class ConstraintManager<H, C> 
        where C : struct, IConstraint
        where H : struct, ConstraintManagerHelper<C> {

    private NativeList<C> constraints;
    private WarmStartManager warmStart;
    public H helper;

    public ConstraintManager() {
        constraints = new NativeList<C>(0, Allocator.Persistent);
        helper = default(H);
        warmStart = new WarmStartManager();
    }

    public void FindConstraints() {
        constraints.Clear();

        helper.FillWithConstraints(constraints);
    }

    public void PreSteps(float dt) {
        for (int j = 0; j < constraints.Length; j++) {
            var c = constraints[j];

            helper.PreStep(ref c, dt, warmStart.GetLambdas(c.id));

            // TODO: Non readonly structs are EVIL
            constraints[j] = c;
        }
    }

    public void ApplyImpulses(float dt) {

        for (int j = 0; j < constraints.Length; j++) {
            var c = constraints[j];

            helper.ApplyImpulse(ref c, dt);

            // TODO: Non readonly structs are EVIL
            constraints[j] = c;
        }
    }

    public void PostSteps() {
        warmStart.SaveLambdas(constraints);
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
        private NativeHashMap<int, Lambdas> prevLambdas;

        public WarmStartManager() {
            prevLambdas = new NativeHashMap<int, Lambdas>(0, Allocator.Persistent);
        }

        public Lambdas GetLambdas(int id) {
            Lambdas lambdas;
            if (CollisionSystem.warmStarting && prevLambdas.TryGetValue(id, out var l)) {
                lambdas = l;
            } else {
                lambdas = new Lambdas();
            }
            return lambdas;
        }

        public void SaveLambdas(NativeArray<C> constraints) {
            prevLambdas.Clear();
            foreach (var constraint in constraints) {
                Debug.Assert(!prevLambdas.ContainsKey(constraint.id), "Duplicate contact id: " + constraint.id.ToString());
                prevLambdas[constraint.id] = constraint.GetAccumulatedLambdas();
            }
        }

        public void Dispose() {
            prevLambdas.Dispose();
        }
    }
}

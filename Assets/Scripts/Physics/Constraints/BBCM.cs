//using Unity.Entities;
//using Unity.Collections;
//using Unity.Mathematics;
//using System.Collections.Generic;
//
//using UnityEngine;
//
//using Physics.Math;
//
//using ContactId = Physics.Math.Geometry.ContactId;
//
//public interface IConstraint {
//    ContactId GetId();
//}
//
//public class WarmStartHelper<C, L> 
//        where C : struct, IConstraint
//        where L : struct {
//    public struct ConstraintWrapper {
//        public C constraint;
//        public L lambdas;
//    }
//    protected NativeList<ConstraintWrapper> constraints;
//    // Maps from contact id to the accumulated lambda of that contact last
//    // frame. Used for warm starting.
//    private NativeHashMap<ContactId, L> prevLambdas;
//
//    public WarmStartHelper() {
//        prevLambdas = new NativeHashMap<ContactId, L>(100, Allocator.Persistent);
//        constraints = new NativeList<ConstraintWrapper>(100, Allocator.Persistent);
//    }
//
//    public void RegisterConstraint(C constraint) {
//        L lambdas;
//        if (CollisionSystem.warmStarting && prevLambdas.TryGetValue(constraint.GetId(), out var l)) {
//            lambdas = l;
//        } else {
//            lambdas = default(L);
//        }
//
//        constraints.Add(new ConstraintWrapper {
//            constraint = constraint,
//            lambdas = lambdas,
//        });
//    }
//
//    public void StoreLambdasAndClearConstraints() {
//        prevLambdas.Clear();
//        foreach (var cw in constraints) {
//            prevLambdas[cw.constraint.GetId()] = cw.lambdas;
//        }
//
//        constraints.Clear();
//    }
//
//    //public void PreStep(ref ComponentDataFromEntity<Box> boxes, float dt) {
//    //    for (int j = 0; j < boxBoxConstraints.Length; j++) {
//    //        var c = boxBoxConstraints[j];
//    //
//    //        Lambdas lambdas;
//    //        if (CollisionSystem.warmStarting && prevLambdas.TryGetValue(c.id, out var l)) {
//    //            lambdas = l;
//    //        } else {
//    //            lambdas = new Lambdas();
//    //        }
//    //
//    //        c.PreStep(ref boxes, dt, lambdas);
//    //
//    //        // TODO: Non readonly structs are EVIL
//    //        boxBoxConstraints[j] = c;
//    //    }
//    //}
//
//    //public void ApplyImpulse(ref ComponentDataFromEntity<Box> boxes, float dt) {
//    //
//    //    for (int j = 0; j < boxBoxConstraints.Length; j++) {
//    //        var c = boxBoxConstraints[j];
//    //
//    //        c.ApplyImpulse(ref boxes, dt);
//    //
//    //        // TODO: Non readonly structs are EVIL
//    //        boxBoxConstraints[j] = c;
//    //    }
//    //}
//
//
//    //public IEnumerable<CollisionSystem.DebugContactInfo> GetContactsForDebug() {
//    //    foreach (var constraint in boxBoxConstraints) {
//    //        yield return new CollisionSystem.DebugContactInfo{normal = constraint.normal, contact = constraint.contact, id = constraint.id};
//    //    }
//    //}
//
//    public void Destroy() {
//        prevLambdas.Dispose();
//        constraints.Dispose();
//    }
//}
//
//
//public class BBCM {
//
//    private ConstraintManager<MyConstraint, Lambdas> cm;
//
//    private struct MyConstraint : IConstraint {
//        public BBC constraint;
//        public Entity e1;
//        public Entity e2;
//        public ContactId id;
//
//        public ContactId GetId() {
//            return id;
//        }
//    }
//
//    public ComponentDataFromEntity<Box> boxes;
//
//    public void FindConstraints(NativeList<Entity> boxEntities) {
//    
//        for (int i = 0; i < boxEntities.Length; i++ ) {
//            for (int j = i+1; j < boxEntities.Length; j++ ) {
//                Entity box1 = boxEntities[i];
//                Entity box2 = boxEntities[j];
//    
//                var manifoldNullable = Geometry.GetIntersectData(
//                    boxes[box1].ToRect(),
//                    boxes[box2].ToRect()
//                );
//    
//                if (manifoldNullable is Geometry.Manifold manifold) {
//    
//
//                    cm.RegisterConstraint(new MyConstraint {
//                        
//                    });
//                    boxBoxConstraints.Add(new ConstraintWrapper(
//                        box1, box2,
//                        manifold, WhichContact.contact1,
//                    ));
//    
//                    if (manifold.contact2 is Geometry.Contact contact) {
//                        //AddConstraint(
//                        //    box1: box1,
//                        //    box2: box2,
//                        //    normal: manifold.normal,
//                        //    contact: contact,
//                        //    boxes: boxes,
//                        //    dt: dt
//                        //);
//    
//                        //boxBoxConstraints.Add(new BoxBoxConstraint(
//                        //    box1, box2,
//                        //    manifold.normal,
//                        //    contact
//                        //));
//    
//                        Debug.Assert(!manifold.contact1.id.Equals(contact.id), "Duplicate contact ids within the same manifold");
//                    }
//                }
//            }
//        }
//    }
//}

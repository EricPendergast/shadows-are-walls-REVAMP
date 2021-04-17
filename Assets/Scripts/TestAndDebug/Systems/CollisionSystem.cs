using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using Physics.Math;
using UnityEngine;

using ContactId = Physics.Math.Geometry.ContactId;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(GravitySystem))]
public class CollisionSystem : SystemBase {
    EntityQuery boxesQuery;
    public static bool accumulateImpulses = true;
    public static bool warmStarting = true;
    public static bool positionCorrection = true;
    // In the future, this will be configurable on a per object basis
    public static float globalFriction = .2f;

    private NativeList<Entity> boxEntities;
    private BoxBoxConstraintManager boxBoxCM;

    protected override void OnCreate() {
        boxEntities = new NativeList<Entity>(100, Allocator.Persistent);
        boxBoxCM = new BoxBoxConstraintManager();
    }

    protected override void OnDestroy() {
        boxEntities.Dispose();
        boxBoxCM.Destroy();
    }

    protected override void OnUpdate() {
        var boxEntities = this.boxEntities;
        StoreBoxEntitiesInto(boxEntities);

        var boxes = GetComponentDataFromEntity<Box>(false);
        boxBoxCM.boxes = boxes;

        boxBoxCM.FindConstraints(boxEntities);

        float dt = Time.DeltaTime;

        boxBoxCM.PreSteps(dt);

        for (int i = 0; i < 10; i++) {
            boxBoxCM.ApplyImpulses(dt);
        }

        boxBoxCM.PostSteps();
    }

    private void StoreBoxEntitiesInto(NativeList<Entity> storeInto) {
        storeInto.Clear();
        storeInto.Length = boxesQuery.CalculateEntityCount();

        Entities
            .WithName("StoreBoxes")
            .WithAll<Box>()
            .WithStoreEntityQueryInField(ref boxesQuery)
            .ForEach((int entityInQueryIndex, ref Entity e) => {
                storeInto[entityInQueryIndex] = e;
            }).Run();
    }

    public struct DebugContactInfo {
        public float2 normal;
        public float2 contact;
        public ContactId id;
    }

    public IEnumerable<DebugContactInfo> GetContactsForDebug() {
        foreach (var item in boxBoxCM.GetContactsForDebug()) {
            yield return item;
        }
    }
}

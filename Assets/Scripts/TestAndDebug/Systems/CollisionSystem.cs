using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using Physics.Math;
using UnityEngine;

using ContactId = Physics.Math.Geometry.ContactId;
using BoxBoxConstraintManager = ConstraintManager<BoxBoxConstraintHelper, StandardConstraint>;
using BoxLightEdgeConstraintManager = ConstraintManager<BoxLightEdgeConstraintHelper, StandardConstraint>;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(GravitySystem))]
public class CollisionSystem : SystemBase {
    EntityQuery boxesQuery;
    EntityQuery lightEdgesQuery;
    public static bool accumulateImpulses = true;
    public static bool warmStarting = true;
    public static bool positionCorrection = true;
    // In the future, this will be configurable on a per object basis
    public static float globalFriction = .2f;

    private NativeList<Entity> boxEntities;
    private NativeList<Entity> lightEdgeEntities;

    private BoxBoxConstraintManager boxBoxCM;
    private BoxLightEdgeConstraintManager boxLightEdgeCM;

    protected override void OnCreate() {
        boxEntities = new NativeList<Entity>(100, Allocator.Persistent);
        lightEdgeEntities = new NativeList<Entity>(100, Allocator.Persistent);

        boxBoxCM = new BoxBoxConstraintManager();
        boxLightEdgeCM = new BoxLightEdgeConstraintManager();
    }

    protected override void OnDestroy() {
        boxEntities.Dispose();
        lightEdgeEntities.Dispose();

        boxBoxCM.Dispose();
        boxLightEdgeCM.Dispose();
    }

    protected override void OnUpdate() {
        var boxEntities = this.boxEntities;
        StoreBoxEntitiesInto(boxEntities);

        var lightEdgeEntities = this.lightEdgeEntities;
        StoreLightEdgeEntitiesInto(lightEdgeEntities);

        var boxes = GetComponentDataFromEntity<Box>(false);
        var lightEdges = GetComponentDataFromEntity<LightEdge>(false);
        var velocities = GetComponentDataFromEntity<Velocity>(false);

        boxBoxCM.helper.Update(
            boxes: boxes,
            boxVels: velocities,
            boxEntities: boxEntities
        );

        boxLightEdgeCM.helper.Update(
            boxes: boxes,
            lightEdges: lightEdges,
            vels: velocities,
            boxEntities: boxEntities,
            lightEdgeEntities: lightEdgeEntities
        );

        boxBoxCM.FindConstraints();
        boxLightEdgeCM.FindConstraints();

        float dt = Time.DeltaTime;

        boxBoxCM.PreSteps(dt);
        boxLightEdgeCM.PreSteps(dt);

        for (int i = 0; i < 10; i++) {
            boxBoxCM.ApplyImpulses(dt);
            boxLightEdgeCM.ApplyImpulses(dt);
        }

        boxBoxCM.PostSteps();
        boxLightEdgeCM.PostSteps();
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

    private void StoreLightEdgeEntitiesInto(NativeList<Entity> storeInto) {
        storeInto.Clear();
        storeInto.Length = lightEdgesQuery.CalculateEntityCount();
    
        Entities
            .WithName("StoreLightEdges")
            .WithAll<LightEdge>()
            .WithStoreEntityQueryInField(ref lightEdgesQuery)
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
        foreach (var item in boxLightEdgeCM.GetContactsForDebug()) {
            yield return item;
        }
    }
}

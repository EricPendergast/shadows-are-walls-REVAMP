using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using Physics.Math;

using Utilities;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(GravitySystem))]
public class ShadowEdgeGenerationSystem : SystemBase {
    private EntityQuery lightSourceQuery;
    private EntityQuery opaqueBoxesQuery;
    private EntityQuery shadHitBoxesQuery;

    public struct ShadowEdgeManifold {
        public Entity shadHitEntity;
        public Physics.Math.Geometry.Contact contact1;
        public Physics.Math.Geometry.Contact? contact2;

        public Entity castingEntity;
        public float2 mount1;
        public float2? mount2;

        public ShapeType castingShapeType;

        public float overlap;
        public float2 normal;
        public float2 lightSource;
    }

    public struct ShadowCornerManifold {
        public Entity castingEntity1;
        public ShapeType castingEntity1Type;
        public float2 e1Mount1;
        public float2? e1Mount2;
        public float2 casting1Corner;

        public Entity castingEntity2;
        public ShapeType castingEntity2Type;
        public float2 e2Mount1;
        public float2? e2Mount2;
        public float2 casting2Corner;

        public Entity lineEntity;
        public bool lineIsShadowEdge;
        public ShapeType lineEntityCastingType;
        // Can be any point on the line
        public float2 linePoint;
        public float2 lineOppositeCorner;


        public float2 normal;
        public int id;
    }

    Dictionary<Entity, ShadowEdgeCalculator> lightManagers;
    NativeList<ShadowEdgeManifold> finalShadowEdgeManifolds;
    NativeList<ShadowCornerManifold> finalShadowCornerManifolds;

    // TODO: This will replace some of the above stuff
    NativeMultiHashMap<Entity, CornerCalculator.Edge> boxOverlappingEdges;
    NativeList<LightSource> lightSources;
    NativeList<AngleCalculator> lightAngleCalculators;

    protected override void OnCreate() {
        lightSourceQuery = GetEntityQuery(typeof(LightSource));
        opaqueBoxesQuery = GetEntityQuery(typeof(Box), typeof(OpaqueObject));
        shadHitBoxesQuery = GetEntityQuery(typeof(Box), typeof(HitShadowsObject));
        lightManagers = new Dictionary<Entity, ShadowEdgeCalculator>();

        finalShadowEdgeManifolds = new NativeList<ShadowEdgeManifold>(Allocator.Persistent);
        finalShadowCornerManifolds = new NativeList<ShadowCornerManifold>(Allocator.Persistent);

        boxOverlappingEdges = new NativeMultiHashMap<Entity, CornerCalculator.Edge>(0, Allocator.Persistent);
        lightSources = new NativeList<LightSource>(Allocator.Persistent);
        lightAngleCalculators = new NativeList<AngleCalculator>(Allocator.Persistent);
    }

    protected override void OnDestroy() {
        Clear(lightManagers);
        finalShadowEdgeManifolds.Dispose();

        boxOverlappingEdges.Dispose();
        lightSources.Dispose();
        lightAngleCalculators.Clear();
    }

    protected override void OnUpdate() {
        // Moving into local scope because required by Entities.ForEach
        var lightSources = this.lightSources;
        var lightAngleCalculators = this.lightAngleCalculators;
        var boxOverlappingEdges = this.boxOverlappingEdges;
        var finalShadowEdgeManifolds = this.finalShadowEdgeManifolds;
        var finalShadowCornerManifolds = this.finalShadowCornerManifolds;

        lightSources.Clear();
        lightAngleCalculators.Clear();
        Entities.ForEach((in LightSource lightSource) => {
            lightSources.Add(lightSource);
            lightAngleCalculators.Add(new AngleCalculator(lightSource));
        }).Run();

        var lightSourceEntities = lightSourceQuery.ToEntityArray(Allocator.TempJob);

        var opaqueBoxes = opaqueBoxesQuery.ToComponentDataArray<Box>(Allocator.TempJob);
        var opaqueBoxEntities = opaqueBoxesQuery.ToEntityArray(Allocator.TempJob);

        var shadHitBoxes = shadHitBoxesQuery.ToComponentDataArray<Box>(Allocator.TempJob);
        var shadHitBoxEntities = shadHitBoxesQuery.ToEntityArray(Allocator.TempJob);

        var boxes = GetComponentDataFromEntity<Box>(true);

        // Step 1: Initialization
        // Can be optimized
        Clear(lightManagers);
        for (int i = 0; i < lightSources.Length; i++) {
            lightManagers[lightSourceEntities[i]] = new ShadowEdgeCalculator(lightSources[i], lightSourceEntities[i], i);
        }


        boxOverlappingEdges.Clear();
        // Step 2: Computing initial contact manifolds
        foreach (var lm in lightManagers.Values) {
            // Compute the sorted shadow list
            // 2.a, 2.b, 2.c
            lm.ComputeManifolds(
                opaqueBoxes, opaqueBoxEntities,
                shadHitBoxes, shadHitBoxEntities,
                ref boxOverlappingEdges);
        }

        // Step 5: Store all non illuminated manifolds

        finalShadowEdgeManifolds.Clear();
        finalShadowCornerManifolds.Clear();

        Entities.WithAll<Box, HitShadowsObject>()
            .ForEach((in Box box, in Entity entity) => {
                var cc = new CornerCalculator(
                    box,
                    entity,
                    lightSources,
                    lightAngleCalculators,
                    It.Iterate(boxOverlappingEdges, entity)
                );

                cc.ComputeManifolds(ref finalShadowEdgeManifolds, ref finalShadowCornerManifolds);
            }).Run();

        lightSourceEntities.Dispose();
        opaqueBoxes.Dispose();
        opaqueBoxEntities.Dispose();
        shadHitBoxes.Dispose();
        shadHitBoxEntities.Dispose();
    }

    public IEnumerable<ShadowEdgeCalculator.ShadowEdgeDebugInfo> GetShadowEdgesForDebug() {
        foreach (var lightManager in lightManagers.Values) {
            foreach (var edge in lightManager.IterShadowEdgeDebugInfo()) {
                yield return edge;
            }
        }
    }

    public List<CornerCalculator.Corner> GetShadowIslandsForDebug() {
        var ret = new List<CornerCalculator.Corner>();
        Entities.WithAll<Box, HitShadowsObject>()
            .WithoutBurst()
            .ForEach((in Box box, in Entity entity) => {
                var cc = new CornerCalculator(
                    box,
                    entity,
                    lightSources,
                    lightAngleCalculators,
                    It.Iterate(boxOverlappingEdges, entity)
                );

                foreach (var item in cc.GetIslandsForDebug()) {
                    ret.Add(item);
                }
            }).Run();
        return ret;
    }

    public List<ShadowCornerManifold> GetCornerManifoldsForDebug() {
        var ret = new List<ShadowCornerManifold>();
        foreach (var item in finalShadowCornerManifolds) {
            ret.Add(item);
        }
        return ret;
    }

    public List<ShadowEdgeManifold> GetEdgeManifoldsForDebug() {
        var ret = new List<ShadowEdgeManifold>();
        foreach (var item in finalShadowEdgeManifolds) {
            ret.Add(item);
        }
        return ret;
    }

    //public IEnumerable<float2> GetRenderPoints(Entity lightSource) {
    //    return lightManagers[lightSource].GetRenderPoints();
    //}

    public NativeList<ShadowEdgeManifold> GetShadowEdgeManifolds() {
        return finalShadowEdgeManifolds;
    }

    private void Clear(Dictionary<Entity, ShadowEdgeCalculator> lightManagers) {
        foreach (var lightManager in lightManagers.Values) {
            lightManager.Dispose();
        }
        lightManagers.Clear();
    }

}

public enum ShapeType : byte {
    Box, Light
}


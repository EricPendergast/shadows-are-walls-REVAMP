using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using Physics.Math;

using Utilities;
// Current plans for this class:
// Generate all manifolds representing contact between a shadow edge and a box, or a light edge and a box.
// These manifolds will be used to generate constraints elsewhere.
//
// Inputs to algorithm:
// - List of opaque boxes
// - List of light sources
// - List of shadowhitting boxes
//
// Global data structures:
// - box_to_manifold: multi map from shadowHit boxes to shadow edge manifolds involving them
// - initial_shadow_corner_manifolds: NativeList of shadow corner manifolds
// Steps of algorithm:
// 1. Ensure each light source has a corresponding LightManager, freshly initialized
// 2. For each light manager
//      2.a Take in all opaque boxes and shadowHitting boxes
//      2.b. Generate a list of (angle, shape) sorted by angle, where an entry
//          (a, s) means "'s' occupies the angular space starting at 'a', and
//          ending at the angle of the next entry"
//      2.c. Use the generated list to calculate manifolds for each shadowEdge, shadowHittingObj pair
//          Put these manifolds in box_to_manifold
//
//  3. For each box with manifolds m1, m2 ... mn
//      3.a. For each pair of manifolds mi, mj
//          3.a.a. If mi and mj intersect to create a shadow corner intersecting their box,
//              add the shadow corner to initial_shadow_corner_manifolds
//
//  4. For each light manager
//      4.a Mark any manifold (in box_to_manifold and
//      initial_shadow_corner_manifolds) illuminated by this light manager as
//      trash
//
//  5. Return all non-trash manifolds
//

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

    public struct CornerManifold {}

    Dictionary<Entity, ShadowEdgeCalculator> lightManagers;
    NativeList<ShadowEdgeManifold> finalShadowEdgeManifolds;
    NativeList<ShadowEdgeManifold> finalLightEdgeManifolds;
    // TODO: Replace this with boxOverlappingEdges
    NativeMultiHashMap<Entity, ShadowEdgeManifold> boxManifolds;

    // TODO: This will replace some of the above stuff
    NativeMultiHashMap<Entity, CornerCalculator.Edge> boxOverlappingEdges;
    NativeList<LightSource> lightSources;
    NativeList<ShadowEdgeCalculator.AngleCalculator> lightAngleCalculators;
    NativeList<CornerManifold> cornerManifolds;

    protected override void OnCreate() {
        lightSourceQuery = GetEntityQuery(typeof(LightSource));
        opaqueBoxesQuery = GetEntityQuery(typeof(Box), typeof(OpaqueObject));
        shadHitBoxesQuery = GetEntityQuery(typeof(Box), typeof(HitShadowsObject));
        lightManagers = new Dictionary<Entity, ShadowEdgeCalculator>();
        finalShadowEdgeManifolds = new NativeList<ShadowEdgeManifold>(Allocator.Persistent);
        finalLightEdgeManifolds = new NativeList<ShadowEdgeManifold>(Allocator.Persistent);
        boxManifolds = new NativeMultiHashMap<Entity, ShadowEdgeManifold>(0, Allocator.Persistent);

        boxOverlappingEdges = new NativeMultiHashMap<Entity, CornerCalculator.Edge>(0, Allocator.Persistent);
        lightSources = new NativeList<LightSource>(Allocator.Persistent);
        lightAngleCalculators = new NativeList<ShadowEdgeCalculator.AngleCalculator>(Allocator.Persistent);
    }

    protected override void OnDestroy() {
        Clear(lightManagers);
        finalShadowEdgeManifolds.Dispose();
        finalLightEdgeManifolds.Dispose();
        boxManifolds.Dispose();

        boxOverlappingEdges.Dispose();
        lightSources.Dispose();
        lightAngleCalculators.Clear();
    }

    protected override void OnUpdate() {
        var lightSources = this.lightSources;
        var lightAngleCalculators = this.lightAngleCalculators;

        lightSources.Clear();
        lightAngleCalculators.Clear();
        Entities.ForEach((in LightSource lightSource) => {
            lightSources.Add(lightSource);
            lightAngleCalculators.Add(new ShadowEdgeCalculator.AngleCalculator(lightSource));
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


        boxManifolds.Clear();
        boxOverlappingEdges.Clear();
        // Step 2: Computing initial contact manifolds
        foreach (var lm in lightManagers.Values) {
            // Compute the sorted shadow list
            // 2.a, 2.b, 2.c
            lm.ComputeManifolds(
                opaqueBoxes, opaqueBoxEntities,
                shadHitBoxes, shadHitBoxEntities,
                ref boxManifolds,
                ref boxOverlappingEdges);
        }

        // Step 5: Store all non illuminated manifolds

        finalShadowEdgeManifolds.Clear();
        finalLightEdgeManifolds.Clear();
        foreach (var kv in boxManifolds) {
            var seManifold = kv.Value;

            switch (seManifold.castingShapeType) {
                case ShapeType.Box:
                    finalShadowEdgeManifolds.Add(seManifold);
                    break;
                case ShapeType.Light:
                    finalLightEdgeManifolds.Add(seManifold);
                    break;
            }
        }

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

    public List<CornerManifold> GetCornerManifoldsForDebug() {
        var ret = new List<CornerManifold>();

        foreach (var m in cornerManifolds) {
            ret.Add(m);
        }

        return ret;
    }

    //public IEnumerable<float2> GetRenderPoints(Entity lightSource) {
    //    return lightManagers[lightSource].GetRenderPoints();
    //}

    public NativeList<ShadowEdgeManifold> GetShadowEdgeManifolds() {
        return finalShadowEdgeManifolds;
    }

    public NativeList<ShadowEdgeManifold> GetLightEdgeManifolds() {
        return finalLightEdgeManifolds;
    }

    private void Clear(Dictionary<Entity, ShadowEdgeCalculator> lightManagers) {
        foreach (var lightManager in lightManagers.Values) {
            lightManager.Dispose();
        }
        lightManagers.Clear();
    }

}

public enum ShapeType {
    Box, Light
}


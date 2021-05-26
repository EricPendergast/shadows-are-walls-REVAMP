using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

using Physics.Math;

using Utilities;

using EdgeMountsMap = Unity.Collections.NativeMultiHashMap<CornerCalculator.Edge.EdgeKey, CornerCalculator.EdgeMount>;
using EdgeMount = CornerCalculator.EdgeMount;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(GravitySystem))]
public class ShadowEdgeGenerationSystem : SystemBase {
    private EntityQuery lightSourceQuery;
    private EntityQuery opaqueBoxesQuery;
    private EntityQuery shadHitBoxesQuery;

    public struct ShadowEdgeManifold {
        public float delta;
        public float2 n;
        // The opaque object
        //public Entity e1;
        public float2 x1;
        public float2 d1;
        // The shadow hitting object
        public Entity e2;
        public float2 x2;

        // Contact point
        public float2 p;
        // Unique id for the point of contact on the shadow hitting object.
        public int contactIdOn2;
    }

    public struct ShadowCornerManifold {
        public float2 x1;
        public float2 d1;
        public float2 x2;
        public float2 d2;
        public float2 x3;
        public float2 s;

        public float2 p;
        public float2 p1;
        public float2 p2;

        public float2 n;
        // This is only used if shape number 3 is not a shadow edge
        public int contactIdOnBox;
    }

    Dictionary<Entity, ShadowEdgeCalculator> lightManagers;

    NativeList<ShadowEdgeConstraint.Partial> partialEdgeConstraints;
    NativeList<ShadowCornerConstraint.Partial> partialCornerConstraints;

    // TODO: This will replace some of the above stuff
    NativeMultiHashMap<Entity, CornerCalculator.Edge> boxOverlappingEdges;
    EdgeMountsMap edgeMounts;

    NativeList<LightSource> lightSources;
    NativeList<AngleCalculator> lightAngleCalculators;

    protected override void OnCreate() {
        lightSourceQuery = GetEntityQuery(typeof(LightSource));
        opaqueBoxesQuery = GetEntityQuery(typeof(Box), typeof(OpaqueObject));
        shadHitBoxesQuery = GetEntityQuery(typeof(Box), typeof(HitShadowsObject));
        lightManagers = new Dictionary<Entity, ShadowEdgeCalculator>();

        partialCornerConstraints = new NativeList<ShadowCornerConstraint.Partial>(Allocator.Persistent);
        partialEdgeConstraints = new NativeList<ShadowEdgeConstraint.Partial>(Allocator.Persistent);

        boxOverlappingEdges = new NativeMultiHashMap<Entity, CornerCalculator.Edge>(0, Allocator.Persistent);
        edgeMounts = new EdgeMountsMap(0, Allocator.Persistent);
        lightSources = new NativeList<LightSource>(Allocator.Persistent);
        lightAngleCalculators = new NativeList<AngleCalculator>(Allocator.Persistent);
    }

    protected override void OnDestroy() {
        Clear(lightManagers);
        partialCornerConstraints.Dispose();
        partialEdgeConstraints.Dispose();

        boxOverlappingEdges.Dispose();
        edgeMounts.Dispose();
        lightSources.Dispose();
        lightAngleCalculators.Clear();
    }

    protected override void OnUpdate() {
        // Moving into local scope because required by Entities.ForEach
        var lightSources = this.lightSources;
        var lightAngleCalculators = this.lightAngleCalculators;
        var boxOverlappingEdges = this.boxOverlappingEdges;
        var edgeMounts = this.edgeMounts;
        var partialEdgeConstraints = this.partialEdgeConstraints;
        var partialCornerConstraints = this.partialCornerConstraints;

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
        edgeMounts.Clear();
        // Step 2: Computing initial contact manifolds
        foreach (var lm in lightManagers.Values) {
            // Compute the sorted shadow list
            // 2.a, 2.b, 2.c
            lm.ComputeManifolds(
                opaqueBoxes, opaqueBoxEntities,
                shadHitBoxes, shadHitBoxEntities,
                ref boxOverlappingEdges,
                ref edgeMounts);
        }

        // Step 5: Store all non illuminated manifolds

        partialCornerConstraints.Clear();
        partialEdgeConstraints.Clear();

        Entities.WithAll<Box, HitShadowsObject>()
            .WithReadOnly(boxOverlappingEdges)
            .WithReadOnly(edgeMounts)
            .ForEach((in Box box, in Entity entity) => {
                var cc = new CornerCalculator(
                    box,
                    entity,
                    lightSources,
                    lightAngleCalculators,
                    It.Iterate(boxOverlappingEdges, entity),
                    ref edgeMounts,
                    new CornerCalculator.Outputs{
                        partialEdgeConstraints = partialEdgeConstraints,
                        partialCornerConstraints = partialCornerConstraints
                    }
                );
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
        // TODO: Put islands in CornerCalculator.Outputs
        Entities.WithAll<Box, HitShadowsObject>()
            .WithoutBurst()
            .ForEach((in Box box, in Entity entity) => {
                var cc = new CornerCalculator(
                    box,
                    entity,
                    lightSources,
                    lightAngleCalculators,
                    It.Iterate(boxOverlappingEdges, entity),
                    ref edgeMounts,
                    new CornerCalculator.Outputs{}
                );

                foreach (var item in cc.GetIslandsForDebug()) {
                    ret.Add(item);
                }
            }).Run();
        return ret;
    }

    public List<ShadowEdgeManifold> GetEdgeManifoldsForDebug() {
        var ret = new List<ShadowEdgeManifold>();
        ComputeCornersForDebug(
            new CornerCalculator.Outputs{
                debugEdgeManifoldCollector = ret
            }
        );
        return ret;
    }

    public List<ShadowCornerManifold> GetCornerManifoldsForDebug() {
        var ret = new List<ShadowCornerManifold>();
        ComputeCornersForDebug(
            new CornerCalculator.Outputs{
                debugCornerManifolds = ret
            }
        );
        return ret;
    }

    public List<(ShadowEdgeManifold, CornerCalculator.EdgeMount)> GetEdgeMountsForDebug() {
        var ret = new List<(ShadowEdgeManifold, CornerCalculator.EdgeMount)>();
        ComputeCornersForDebug(
            new CornerCalculator.Outputs{
                debugEdgeMounts = ret
            }
        );
        return ret;
    }

    public List<(ShadowCornerManifold, CornerCalculator.EdgeMount, CornerCalculator.EdgeMount)> GetCornerMountsForDebug() {
        var ret = new List<(ShadowCornerManifold, CornerCalculator.EdgeMount, CornerCalculator.EdgeMount)>();
        ComputeCornersForDebug(
            new CornerCalculator.Outputs{
                debugCornerMounts = ret
            }
        );
        return ret;
    }

    private void ComputeCornersForDebug(CornerCalculator.Outputs outputs) {
        Entities.WithAll<Box, HitShadowsObject>()
            .WithoutBurst()
            .ForEach((in Box box, in Entity entity) => {
                new CornerCalculator(
                    box,
                    entity,
                    lightSources,
                    lightAngleCalculators,
                    It.Iterate(boxOverlappingEdges, entity),
                    ref edgeMounts,
                    outputs
                );
            }).Run();
    }

    public NativeList<ShadowEdgeConstraint.Partial> GetPartialEdgeConstraints() {
        return partialEdgeConstraints;
    }

    public NativeList<ShadowCornerConstraint.Partial> GetPartialCornerConstraints() {
        return partialCornerConstraints;
    }

    //public IEnumerable<float2> GetRenderPoints(Entity lightSource) {
    //    return lightManagers[lightSource].GetRenderPoints();
    //}

    private void Clear(Dictionary<Entity, ShadowEdgeCalculator> lightManagers) {
        foreach (var lightManager in lightManagers.Values) {
            lightManager.Dispose();
        }
        lightManagers.Clear();
    }

}

// TODO: Rename this to EdgeSource
public enum ShapeType : byte {
    Box, Light
}


using Unity.Entities;
using Unity.Collections;
using System.Collections.Generic;

using Utilities;

using EdgeMountsMap = Unity.Collections.NativeMultiHashMap<ShadowCornerCalculator.Edge.EdgeKey, ShadowCornerCalculator.EdgeMount>;

[UpdateInGroup(typeof(ConstraintGenerationSystemGroup))]
public class ShadowEdgeGenerationSystem : SystemBase {
    private EntityQuery lightSourceQuery;
    private EntityQuery opaqueBoxesQuery;
    private EntityQuery shadHitBoxesQuery;

    Dictionary<Entity, ShadowEdgeCalculator> lightManagers;

    NativeList<TwoWayPenConstraint.Partial> partialEdgeConstraints;
    NativeList<ThreeWayPenConstraint.Partial> partialCornerConstraints;

    NativeMultiHashMap<Entity, ShadowCornerCalculator.Edge> boxOverlappingEdges;
    EdgeMountsMap edgeMounts;

    NativeList<LightSource> lightSources;
    NativeList<AngleCalculator> lightAngleCalculators;

    protected override void OnCreate() {
        lightSourceQuery = GetEntityQuery(typeof(LightSource));
        opaqueBoxesQuery = GetEntityQuery(typeof(Box), typeof(OpaqueObject));
        shadHitBoxesQuery = GetEntityQuery(typeof(Box), typeof(HitShadowsObject));
        lightManagers = new Dictionary<Entity, ShadowEdgeCalculator>();

        partialCornerConstraints = new NativeList<ThreeWayPenConstraint.Partial>(Allocator.Persistent);
        partialEdgeConstraints = new NativeList<TwoWayPenConstraint.Partial>(Allocator.Persistent);

        boxOverlappingEdges = new NativeMultiHashMap<Entity, ShadowCornerCalculator.Edge>(0, Allocator.Persistent);
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
                var cc = new ShadowCornerCalculator(
                    box,
                    entity,
                    lightSources,
                    lightAngleCalculators,
                    It.Iterate(boxOverlappingEdges, entity),
                    ref edgeMounts,
                    new ShadowCornerCalculator.Outputs{
                        partialEdgeConstraints = partialEdgeConstraints,
                        partialCornerConstraints = partialCornerConstraints
                    }
                );
            }).Run();

        {
            var masses = GetComponentDataFromEntity<Mass>();

            var edgeConstraintsOut = World.GetOrCreateSystem<CollisionSystem>().GetTwoWayPenConstraintsInput();

            float dt = Time.DeltaTime;

            foreach (var pec in partialEdgeConstraints) {
                edgeConstraintsOut.Add(new TwoWayPenConstraint(pec, masses, dt));
            }

            var cornerConstraintsOut = World.GetOrCreateSystem<CollisionSystem>().GetThreeWayPenConstraintsInput();

            foreach (var pcc in partialCornerConstraints) {
                cornerConstraintsOut.Add(new ThreeWayPenConstraint(pcc, masses, dt));
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

    public List<ShadowCornerCalculator.Corner> GetShadowIslandsForDebug() {
        var ret = new List<ShadowCornerCalculator.Corner>();
        // TODO: Put islands in CornerCalculator.Outputs
        Entities.WithAll<Box, HitShadowsObject>()
            .WithoutBurst()
            .ForEach((in Box box, in Entity entity) => {
                var cc = new ShadowCornerCalculator(
                    box,
                    entity,
                    lightSources,
                    lightAngleCalculators,
                    It.Iterate(boxOverlappingEdges, entity),
                    ref edgeMounts,
                    new ShadowCornerCalculator.Outputs{}
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
            new ShadowCornerCalculator.Outputs{
                debugEdgeManifoldCollector = ret
            }
        );
        return ret;
    }

    public List<ShadowCornerManifold> GetCornerManifoldsForDebug() {
        var ret = new List<ShadowCornerManifold>();
        ComputeCornersForDebug(
            new ShadowCornerCalculator.Outputs{
                debugCornerManifolds = ret
            }
        );
        return ret;
    }

    public List<(ShadowEdgeManifold, ShadowCornerCalculator.EdgeMount)> GetEdgeMountsForDebug() {
        var ret = new List<(ShadowEdgeManifold, ShadowCornerCalculator.EdgeMount)>();
        ComputeCornersForDebug(
            new ShadowCornerCalculator.Outputs{
                debugEdgeMounts = ret
            }
        );
        return ret;
    }

    public List<ShadowCornerCalculator.Outputs.CornerMountTuple> GetCornerMountsForDebug() {
        var ret = new List<ShadowCornerCalculator.Outputs.CornerMountTuple>();
        ComputeCornersForDebug(
            new ShadowCornerCalculator.Outputs{
                debugCornerMounts = ret
            }
        );
        return ret;
    }

    private void ComputeCornersForDebug(ShadowCornerCalculator.Outputs outputs) {
        Entities.WithAll<Box, HitShadowsObject>()
            .WithoutBurst()
            .ForEach((in Box box, in Entity entity) => {
                new ShadowCornerCalculator(
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

    private void Clear(Dictionary<Entity, ShadowEdgeCalculator> lightManagers) {
        foreach (var lightManager in lightManagers.Values) {
            lightManager.Dispose();
        }
        lightManagers.Clear();
    }
}

public enum EdgeSourceType : byte {
    Box, Light
}

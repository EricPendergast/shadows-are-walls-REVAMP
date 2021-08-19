using Unity.Entities;
using Unity.Collections;
using System.Collections.Generic;

using Utilities;

using EdgeMountsMap = Unity.Collections.NativeMultiHashMap<ShadowCornerCalculator.Edge.EdgeKey, ShadowCornerCalculator.EdgeMount>;

[UpdateInGroup(typeof(ContactGenerationGroup))]
public class ShadowConstraintSystem : SystemBase {
    private EntityQuery lightSourceQuery;
    private EntityQuery opaqueBoxesQuery;
    private EntityQuery shadHitBoxesQuery;

    Dictionary<Entity, ShadowEdgeCalculatorClassWrapper> shadowEdgeCalculators;

    NativeList<TwoWayPenFricConstraint.Partial> partialEdgeConstraints;
    NativeList<ThreeWayPenConstraint.Partial> partialCornerConstraints;

    NativeMultiHashMap<Entity, ShadowContactStore> shadowContactStores;

    NativeMultiHashMap<Entity, ShadowCornerCalculator.Edge> boxOverlappingEdges;
    EdgeMountsMap edgeMounts;

    NativeList<LightSource> lightSources;
    NativeList<AngleCalculator> lightAngleCalculators;

    protected override void OnCreate() {
        lightSourceQuery = GetEntityQuery(typeof(LightSource), typeof(Position));
        opaqueBoxesQuery = GetEntityQuery(typeof(Box), typeof(Position), typeof(OpaqueObject));
        shadHitBoxesQuery = GetEntityQuery(typeof(Box), typeof(Position), typeof(HitShadowsObject));
        shadowEdgeCalculators = new Dictionary<Entity, ShadowEdgeCalculatorClassWrapper>();

        partialCornerConstraints = new NativeList<ThreeWayPenConstraint.Partial>(Allocator.Persistent);
        partialEdgeConstraints = new NativeList<TwoWayPenFricConstraint.Partial>(Allocator.Persistent);

        shadowContactStores = new NativeMultiHashMap<Entity, ShadowContactStore>(0, Allocator.Persistent);

        boxOverlappingEdges = new NativeMultiHashMap<Entity, ShadowCornerCalculator.Edge>(0, Allocator.Persistent);
        edgeMounts = new EdgeMountsMap(0, Allocator.Persistent);
        lightSources = new NativeList<LightSource>(Allocator.Persistent);
        lightAngleCalculators = new NativeList<AngleCalculator>(Allocator.Persistent);
    }

    protected override void OnDestroy() {
        Clear(shadowEdgeCalculators);
        partialCornerConstraints.Dispose();
        partialEdgeConstraints.Dispose();
        
        shadowContactStores.Dispose();

        boxOverlappingEdges.Dispose();
        edgeMounts.Dispose();
        lightSources.Dispose();
        lightAngleCalculators.Dispose();
    }

    protected override void OnUpdate() {
        // Moving into local scope because required by Entities.ForEach
        var lightSources = this.lightSources;
        var lightAngleCalculators = this.lightAngleCalculators;
        var boxOverlappingEdges = this.boxOverlappingEdges;
        var edgeMounts = this.edgeMounts;
        var partialEdgeConstraints = this.partialEdgeConstraints;
        var partialCornerConstraints = this.partialCornerConstraints;
        var shadowContactStores = this.shadowContactStores;

        lightSources.Clear();
        lightAngleCalculators.Clear();
        Entities.ForEach((in LightSource lightSource, in Position pos) => {
            lightSources.Add(lightSource);
            lightAngleCalculators.Add(new AngleCalculator(lightSource, pos));
        }).Run();

        var lightSourceEntities = lightSourceQuery.ToEntityArray(Allocator.TempJob);

        Clear(shadowEdgeCalculators);
        for (int i = 0; i < lightSources.Length; i++) {
            shadowEdgeCalculators[lightSourceEntities[i]] = new ShadowEdgeCalculatorClassWrapper(lightSources[i], lightSourceEntities[i], lightAngleCalculators[i], i);
        }

        boxOverlappingEdges.Clear();
        edgeMounts.Clear();

        ComputeShadowEdges(
            new ShadowEdgeCalculator.Emitter {
                boxOverlappingEdges = boxOverlappingEdges,
                edgeMounts = edgeMounts
            }, useBurst: true
        );

        partialCornerConstraints.Clear();
        partialEdgeConstraints.Clear();
        shadowContactStores.Clear();
        var shadowContactStoresWriter = shadowContactStores.AsParallelWriter();
        var knownShadowContactStores = GetBufferFromEntity<ShadowContactStore>(isReadOnly:true);

        Entities
            .WithAll<HitShadowsObject>()
            .WithReadOnly(boxOverlappingEdges)
            .WithReadOnly(edgeMounts)
            .WithReadOnly(knownShadowContactStores)
            .ForEach((in Box box, in Position pos, in Entity entity) => {
                var cc = new ShadowCornerCalculator(
                    box,
                    pos,
                    entity,
                    new ShadowCornerCalculator.Env{
                        lights = lightSources,
                        lightAngleCalculators = lightAngleCalculators,
                        edges = It.Iterate(boxOverlappingEdges, entity),
                        edgeMounts = edgeMounts,
                        knownShadowContactStores = knownShadowContactStores
                    },
                    new ShadowCornerCalculator.Outputs{
                        partialEdgeConstraints = partialEdgeConstraints,
                        partialCornerConstraints = partialCornerConstraints,
                        shadowContactStores = shadowContactStores
                    }
                );
            }).Run();

        Entities
            .WithReadOnly(shadowContactStores)
            .ForEach((Entity e, ref DynamicBuffer<ShadowContactStore> contacts) => {
                contacts.Clear();
                foreach (var contact in It.Iterate(shadowContactStores, e)) {
                    contacts.Add(contact);
                }
            }).Run();

        {
            var masses = GetComponentDataFromEntity<Mass>(isReadOnly: true);
            var frictions = GetComponentDataFromEntity<Friction>(isReadOnly: true);

            var edgeConstraintsOut = World.GetOrCreateSystem<ConstraintGatherSystem>().GetTwoWayPenFricConstraintsInput();

            float dt = Time.DeltaTime;

            foreach (var pec in partialEdgeConstraints) {
                var f1 = frictions.HasComponent(pec.e1) ? frictions[pec.e1].friction : 0;
                var f2 = frictions.HasComponent(pec.e2) ? frictions[pec.e2].friction : 0;
                edgeConstraintsOut.Add(new TwoWayPenFricConstraint(pec, masses, dt: dt, friction: f1*f2));
            }

            var cornerConstraintsOut = World.GetOrCreateSystem<ConstraintGatherSystem>().GetThreeWayPenConstraintsInput();

            foreach (var pcc in partialCornerConstraints) {
                cornerConstraintsOut.Add(new ThreeWayPenConstraint(pcc, masses, dt));
            }
        }

        lightSourceEntities.Dispose();
    }

    private void ComputeShadowEdges(ShadowEdgeCalculator.Emitter emitter, bool useBurst) {
        var shadowEdgeCalculatorEnv = new ShadowEdgeCalculator.Env {
            opaqueBoxes = opaqueBoxesQuery.ToComponentDataArray<Box>(Allocator.TempJob),
            opaquePositions = opaqueBoxesQuery.ToComponentDataArray<Position>(Allocator.TempJob),
            opaqueBoxEntities = opaqueBoxesQuery.ToEntityArray(Allocator.TempJob),

            shadHitBoxes = shadHitBoxesQuery.ToComponentDataArray<Box>(Allocator.TempJob),
            shadHitPositions = shadHitBoxesQuery.ToComponentDataArray<Position>(Allocator.TempJob),
            shadHitBoxEntities = shadHitBoxesQuery.ToEntityArray(Allocator.TempJob)
        };

        foreach (var lm in shadowEdgeCalculators.Values) {
            var shadowEdgeCalculator = lm.shadowEdgeCalculator;
            if (useBurst) {
                Job.WithBurst().WithCode(() => {
                    shadowEdgeCalculator.ComputeShadowEdgeContacts(
                        shadowEdgeCalculatorEnv,
                        emitter
                    );
                }).Run();
            } else {
                Job.WithoutBurst().WithCode(() => {
                    shadowEdgeCalculator.ComputeShadowEdgeContacts(
                        shadowEdgeCalculatorEnv,
                        emitter
                    );
                }).Run();
            }

            lm.shadowEdgeCalculator = shadowEdgeCalculator;
        }

        shadowEdgeCalculatorEnv.Dispose();
    }

    public IEnumerable<ShadowEdgeCalculator.ShadowEdgeDebugInfo> GetShadowEdgesForDebug() {
        ShadowEdgeCalculator.Emitter.shadowEdgeDebugInfo = new List<ShadowEdgeCalculator.ShadowEdgeDebugInfo>();

        ComputeShadowEdges(new ShadowEdgeCalculator.Emitter{}, useBurst: false);

        foreach (var info in ShadowEdgeCalculator.Emitter.shadowEdgeDebugInfo) {
            yield return info;
        }

        ShadowEdgeCalculator.Emitter.shadowEdgeDebugInfo = null;
    }

    public List<ShadowCornerCalculator.Corner> GetShadowIslandsForDebug() {
        var ret = new List<ShadowCornerCalculator.Corner>();

        var emitter = new ShadowEdgeCalculator.Emitter { 
            boxOverlappingEdges = new NativeMultiHashMap<Entity, ShadowCornerCalculator.Edge>(0, Allocator.TempJob),
            edgeMounts = new EdgeMountsMap(0, Allocator.TempJob)
        };
        ComputeShadowEdges(emitter, useBurst: false);
        var boxOverlappingEdges = emitter.boxOverlappingEdges.Value;
        var edgeMounts = emitter.edgeMounts.Value;

        // TODO: Put islands in CornerCalculator.Outputs
        Entities.WithAll<Box, HitShadowsObject>()
            .WithoutBurst()
            .ForEach((in Box box, in Position pos, in Entity entity) => {
                var cc = new ShadowCornerCalculator(
                    box,
                    pos,
                    entity,
                    new ShadowCornerCalculator.Env{
                        lights = lightSources,
                        lightAngleCalculators = lightAngleCalculators,
                        edges = It.Iterate(boxOverlappingEdges, entity),
                        edgeMounts = edgeMounts
                    },
                    new ShadowCornerCalculator.Outputs{}
                );

                foreach (var item in cc.GetIslandsForDebug()) {
                    ret.Add(item);
                }
            }).Run();

        emitter.boxOverlappingEdges.Value.Dispose();
        emitter.edgeMounts.Value.Dispose();
        return ret;
    }

    public List<ShadowEdgeManifold> GetEdgeManifoldsForDebug() {
        var ret = new List<ShadowEdgeManifold>();

        ShadowCornerCalculator.Outputs.debugEdgeManifoldCollector = ret;
        ComputeCornersForDebug(
            new ShadowCornerCalculator.Outputs{}
        );
        ShadowCornerCalculator.Outputs.debugEdgeManifoldCollector = null;

        return ret;
    }

    public List<ShadowCornerManifold> GetCornerManifoldsForDebug() {
        var ret = new List<ShadowCornerManifold>();

        ShadowCornerCalculator.Outputs.debugCornerManifolds = ret;
        ComputeCornersForDebug(
            new ShadowCornerCalculator.Outputs{}
        );
        ShadowCornerCalculator.Outputs.debugCornerManifolds = null;

        return ret;
    }

    public List<ShadowCornerCalculator.Outputs.EdgeMountTuple> GetEdgeMountsForDebug() {
        var ret = new List<ShadowCornerCalculator.Outputs.EdgeMountTuple>();

        ShadowCornerCalculator.Outputs.debugEdgeMounts = ret;
        ComputeCornersForDebug(
            new ShadowCornerCalculator.Outputs{}
        );
        ShadowCornerCalculator.Outputs.debugEdgeMounts = null;

        return ret;
    }

    public List<ShadowCornerCalculator.Outputs.CornerMountTuple> GetCornerMountsForDebug() {
        var ret = new List<ShadowCornerCalculator.Outputs.CornerMountTuple>();

        ShadowCornerCalculator.Outputs.debugCornerMounts = ret;
        ComputeCornersForDebug(
            new ShadowCornerCalculator.Outputs{
            }
        );
        ShadowCornerCalculator.Outputs.debugCornerMounts = null;

        return ret;
    }

    private void ComputeCornersForDebug(ShadowCornerCalculator.Outputs outputs) {
        Entities.WithAll<Box, HitShadowsObject>()
            .WithoutBurst()
            .ForEach((in Box box, in Position pos, in Entity entity) => {
                new ShadowCornerCalculator(
                    box,
                    pos,
                    entity,
                    new ShadowCornerCalculator.Env{
                        lights = lightSources,
                        lightAngleCalculators = lightAngleCalculators,
                        edges = It.Iterate(boxOverlappingEdges, entity),
                        edgeMounts = edgeMounts
                    },
                    outputs
                );
            }).Run();
    }

    private void Clear(Dictionary<Entity, ShadowEdgeCalculatorClassWrapper> shadowEdgeCalculators) {
        foreach (var lightManager in shadowEdgeCalculators.Values) {
            lightManager.Dispose();
        }
        shadowEdgeCalculators.Clear();
    }
}

public enum EdgeSourceType : byte {
    Box, Light
}


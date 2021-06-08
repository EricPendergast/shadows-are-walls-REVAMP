using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;

using UnityEngine;

using Physics.Math;

[UpdateInGroup(typeof(ConstraintGenerationSystemGroup))]
public class DirectConstraintSystem : SystemBase {
    EntityQuery boxesQuery;
    ComponentDataFromEntity<Box> boxes;
    ComponentDataFromEntity<Mass> masses;
    float dt;

    protected override void OnCreate() {
        boxesQuery = GetEntityQuery(typeof(Box));
    }

    protected override void OnUpdate() {
        NativeArray<Entity> boxEntities = boxesQuery.ToEntityArray(Allocator.TempJob);
        dt = Time.DeltaTime;
        boxes = GetComponentDataFromEntity<Box>();
        masses = GetComponentDataFromEntity<Mass>();

        var constraints = World.GetOrCreateSystem<CollisionSystem>().GetStandardConstraintsInput();

        for (int i = 0; i < boxEntities.Length; i++ ) {
            for (int j = i+1; j < boxEntities.Length; j++ ) {
                Entity box1 = boxEntities[i];
                Entity box2 = boxEntities[j];

                var manifoldNullable = GetManifold(box1, box2);

                if (manifoldNullable is Geometry.Manifold manifold) {


                    AddConstraint(ref constraints, box1, box2, manifold, true);

                    if (manifold.contact2 is Geometry.Contact contact) {

                        AddConstraint(ref constraints, box1, box2, manifold, false);
                        Debug.Assert(!manifold.contact1.id.Equals(contact.id), "Duplicate contact ids within the same manifold");
                    }
                }
            }
        }

        boxEntities.Dispose();
    }

    private void AddConstraint(ref NativeList<TwoWayPenFricConstraint> constraints, Entity e1, Entity e2, Geometry.Manifold manifold, bool useContact1) {
        Box box1 = boxes[e1];
        Box box2 = boxes[e2];

        if (masses[e1].mass == math.INFINITY && masses[e2].mass == math.INFINITY) {
            return;
        }

        var partial = new TwoWayPenFricConstraint.Partial(
            e1, e2,
            box1, box2,
            manifold,
            useContact1
        );

        constraints.Add(new TwoWayPenFricConstraint(partial, masses, dt));
    }

    private Geometry.Manifold? GetManifold(Entity box1, Entity box2) {
        return Geometry.GetIntersectData(
            boxes[box1].ToRect(),
            boxes[box2].ToRect()
        );
    }
}

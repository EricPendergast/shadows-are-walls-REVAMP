using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

public class EdgeMountsGizmoDrawer : MonoBehaviour {
    public float drawRadius = .3f;
    public bool enable = true;
    void OnDrawGizmos() {
        if (Application.isPlaying && enable) {
            var world = World.DefaultGameObjectInjectionWorld;

            var mountCount = new Dictionary<float2, int>();

            foreach (var p in world.GetOrCreateSystem<ShadowEdgeGenerationSystem>().GetEdgeMountsForDebug()) {
                var (manifold, mount) = p;
                int id = Mathf.Abs(manifold.id.GetHashCode());
                int thisMountCount;
                if (mountCount.ContainsKey(mount.point)) {
                    thisMountCount = mountCount[mount.point]+1;
                } else {
                    thisMountCount = 1;
                }

                mountCount[mount.point] = thisMountCount;
                
                Gizmos.color = new Color(
                        (id % 4591 % 256)/256.0f, 
                        (id % 5347 % 256)/265.0f,
                        (id % 3797 % 256)/265.0f);
                Gizmos.DrawSphere((Vector2)mount.point, drawRadius*math.pow(.9f, thisMountCount));
                Gizmos.DrawLine((Vector2)mount.point, (Vector2)mount.shapeCenter);
            }
        }
    }
}

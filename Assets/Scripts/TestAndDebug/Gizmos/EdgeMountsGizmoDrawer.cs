using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

public class EdgeMountsGizmoDrawer : MonoBehaviour {
    public bool enable = true;
    public float drawRadius = .3f;
    public bool renderMountsWithManifoldColor = true;

    public bool renderEdgeMounts = true;
    public bool renderCornerMounts = true;

    public bool drawManifoldS = false;

    void OnDrawGizmos() {
        if (Application.isPlaying && enable) {
            var world = World.DefaultGameObjectInjectionWorld;

            var mountCount = new Dictionary<float2, int>();

            int RegisterMount(ShadowCornerCalculator.EdgeMount mount) {
                int thisMountCount;
                if (mountCount.ContainsKey(mount.point)) {
                    thisMountCount = mountCount[mount.point]+1;
                } else {
                    thisMountCount = 1;
                }
                mountCount[mount.point] = thisMountCount;

                return thisMountCount;
            }

            void DrawMount(ShadowCornerCalculator.EdgeMount mount, int id) {
                int thisMountCount = RegisterMount(mount);
                
                Gizmos.color = new Color(
                        (id % 4591 % 256)/256.0f, 
                        (id % 5347 % 256)/265.0f,
                        (id % 3797 % 256)/265.0f);
                Gizmos.DrawSphere((Vector2)mount.point, drawRadius*math.pow(.9f, thisMountCount));
                Gizmos.DrawLine((Vector2)mount.point, (Vector2)mount.shapeCenter);
            }


            if (renderEdgeMounts) {
                foreach (var (manifold, mount, _) in world.GetOrCreateSystem<ShadowConstraintSystem>().GetEdgeMountsForDebug()) {
                    int id = renderMountsWithManifoldColor ? Mathf.Abs(manifold.contactIdOn2) : Mathf.Abs(mount.id);
                    DrawMount(mount, id);
                }
            }

            if (renderCornerMounts) {
                foreach (var (manifold, mount1, mount2, mount3, _) in world.GetOrCreateSystem<ShadowConstraintSystem>().GetCornerMountsForDebug()) {
                    int id1 = Mathf.Abs(mount1.id);
                    int id2 = Mathf.Abs(mount2.id);
                    int id3 = mount3 != null ? Mathf.Abs(mount3.Value.id) : 0;

                    if (renderMountsWithManifoldColor) {
                        id1 = id2 = id3 = Mathf.Abs(manifold.contactIdOnBox);
                    }

                    DrawMount(mount1, id1);
                    DrawMount(mount2, id2);
                    if (mount3 != null) {
                        DrawMount(mount3.Value, id3);
                    }

                    if (drawManifoldS) {
                        Gizmos.color = Color.red;
                        Gizmos.DrawSphere((Vector2)manifold.s, .3f);
                    }
                }
            }
        }
    }
}

using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Physics.Math;
using Unity.Mathematics;

public class CornerContactsGizmoDrawer : MonoBehaviour {
    public bool enable = true;
    void OnDrawGizmos() {
        if (Application.isPlaying && enable) {
            var world = World.DefaultGameObjectInjectionWorld;

            var set = new HashSet<int>();

            foreach (var manifold in world.GetOrCreateSystem<ShadowEdgeGenerationSystem>().GetCornerManifoldsForDebug()) {
                //void DrawContact(Geometry.Contact contact) {
                //    Gizmos.color = Color.red;
                //    Gizmos.DrawRay((Vector2)contact.point, (Vector2)manifold.normal);
                //    int id = Mathf.Abs(contact.id.GetHashCode());
                //    // Indicates there is a duplicate contact. This should never happen
                //    float m = set.Contains(id) ? 5 : 1;
                //    set.Add(contact.id);
                //    
                //    Gizmos.color = new Color(
                //            (id % 4591 % 256)/256.0f, 
                //            (id % 5347 % 256)/265.0f,
                //            (id % 3797 % 256)/265.0f);
                //    Gizmos.DrawSphere((Vector2)contact.point, .05f*m);
                //}

                //DrawContact(manifold.contact1);
                //if (manifold.contact2 is Geometry.Contact c) {
                //    DrawContact(c);
                //}
                void DrawPoint(Vector2 point) {
                    float radius = .05f;
                    Gizmos.DrawSphere(point, radius);
                    Gizmos.DrawRay(point, (Vector2)manifold.normal);
                }

                Gizmos.color = Color.red;
                DrawPoint(manifold.casting1Corner);
                DrawPoint(manifold.casting2Corner);
                DrawPoint(manifold.lineOppositeCorner);
            }
        }
    }
}



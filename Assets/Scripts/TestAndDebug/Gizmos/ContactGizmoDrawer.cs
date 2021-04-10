using UnityEngine;
using Unity.Entities;

public class ContactGizmoDrawer : MonoBehaviour {
    void OnDrawGizmos() {
        if (Application.isPlaying) {
            var world = World.DefaultGameObjectInjectionWorld;

            Gizmos.color = Color.red;
            foreach (var contact in world.GetOrCreateSystem<CollisionSystem>().GetContactsForDebug()) {
                Gizmos.DrawSphere((Vector2)contact.contact, .05f);
                Gizmos.DrawRay((Vector2)contact.contact, (Vector2)contact.normal);
            }
        }
    }
}



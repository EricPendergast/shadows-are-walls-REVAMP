using UnityEngine;
using Unity.Entities;

public class ContactGizmoDrawer : MonoBehaviour {
    void OnDrawGizmos() {
        if (Application.isPlaying) {
            var world = World.DefaultGameObjectInjectionWorld;

            foreach (var contact in world.GetOrCreateSystem<CollisionSystem>().GetContactsForDebug()) {
                Gizmos.color = Color.red;
                //Gizmos.DrawSphere((Vector2)contact.contact, .01f);
                Gizmos.DrawRay((Vector2)contact.contact, (Vector2)contact.normal);

                Gizmos.color = new Color(
                        (contact.id % 4591 % 256)/256.0f, 
                        (contact.id % 5347 % 256)/265.0f,
                        (contact.id % 3797 % 256)/265.0f);
                Gizmos.DrawSphere((Vector2)contact.contact, .05f);
            }
        }
    }
}



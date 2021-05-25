using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;

public class ContactGizmoDrawer : MonoBehaviour {
    public bool enable = true;
    void OnDrawGizmos() {
        if (Application.isPlaying && enable) {
            var world = World.DefaultGameObjectInjectionWorld;

            var set = new HashSet<int>();

            Debug.LogError("ContactGizmoDrawer is currently broken");

            //foreach (var contact in world.GetOrCreateSystem<CollisionSystem>().GetContactsForDebug()) {
            //    // Indicates there is a duplicate contact. This should never happen
            //    float m = set.Contains(contact.id) ? 5 : 1;
            //    set.Add(contact.id);
            //    
            //    Gizmos.color = Color.red;
            //    //Gizmos.DrawSphere((Vector2)contact.contact, .01f);
            //    Gizmos.DrawRay((Vector2)contact.contact, (Vector2)contact.normal);
            //
            //    int id = Mathf.Abs(contact.id.GetHashCode());
            //
            //    Gizmos.color = new Color(
            //            (id % 4591 % 256)/256.0f, 
            //            (id % 5347 % 256)/265.0f,
            //            (id % 3797 % 256)/265.0f);
            //    Gizmos.DrawSphere((Vector2)contact.contact, .05f*m);
            //}
        }
    }
}



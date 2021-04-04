using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

[ExecuteAlways]
public class TestLineIntersection : MonoBehaviour {

    public Vector2 p1;
    public Vector2 p2;
    public Vector2 p3;
    public Vector2 p4;

    public float2 intersection;

    void OnDrawGizmosSelected() {
        Gizmos.color = Color.red;
        Gizmos.DrawLine(p1, p2);
        Gizmos.DrawSphere(p1, .1f);
        Gizmos.DrawSphere(p2, .1f);

        Gizmos.color = Color.green;
        Gizmos.DrawLine(p3, p4);
        Gizmos.DrawSphere(p3, .1f);
        Gizmos.DrawSphere(p4, .1f);

        Gizmos.color = Color.black;
        intersect(p1, p2, p3, p4, out intersection);
        Gizmos.DrawSphere((Vector2)intersection, .1f);

    }

    void Update() {
        if (!MouseMove(ref p1)){}
        else if (!MouseMove(ref p2)){}
        else if (!MouseMove(ref p3)){}
        else if (!MouseMove(ref p4)){}
    }

    float drawRadius = .5f;
    bool MouseMove(ref Vector2 point) {
        Vector2 worldPosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);
        if (Input.GetMouseButton(0)) {
            if (((Vector2)point - worldPosition).magnitude < drawRadius) {
                point = worldPosition;
                return true;
            }
        }
        return false;
    }
    public static void intersect(float2 p1, float2 p2, float2 p3, float2 p4, out float2 intersection) {
        float x1 = p1.x;
        float x2 = p2.x;
        float x3 = p3.x;
        float x4 = p4.x;

        float y1 = p1.y;
        float y2 = p2.y;
        float y3 = p3.y;
        float y4 = p4.y;

        float D = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);

        float Px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4))/D;
        float Py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4))/D;

        intersection.x = Px;
        intersection.y = Py;
    }
}

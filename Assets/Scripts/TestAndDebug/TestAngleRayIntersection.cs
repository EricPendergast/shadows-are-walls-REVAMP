using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class TestAngleRayIntersection : MonoBehaviour {
    public Vector2 p1;
    public Vector2 p2;
    public float theta1;
    public float theta2;

    public Vector2 intersection;
    public float h;
    public float l;
    public float iprimex;

    void OnDrawGizmosSelected() {
        Gizmos.color = Color.red;
        Gizmos.DrawRay(p1, rotate(Vector2.right, theta1)*1000);
        Gizmos.DrawSphere(p1, .1f);

        Gizmos.color = Color.green;
        Gizmos.DrawRay(p2, rotate(Vector2.right, theta2)*1000);
        Gizmos.DrawSphere(p2, .1f);

        Gizmos.color = Color.black;
        intersection = intersect(p1, p2, theta1, theta2);
        Gizmos.DrawSphere((Vector2)intersection, .1f);

    }

    void Update() {
        
    }

    float2 intersect(float2 p1, float2 p2, float theta1, float theta2) {
        //return math.mul(R(theta1), new float2(
        //        (math.mul(R(-theta1), (p2-p1)).x
        //            + math.tan(math.PI/2 - (theta2-theta1))*
        //                ((math.mul(R(-theta2), (p2-p1))).y)
        //        ), 0));

        h = math.abs((math.mul(R(-theta1), p2 - p1)).y);
        l = math.tan(math.PI/2 - (theta2 - theta1))*h;
        iprimex = math.mul(R(-theta1), p2 - p1).x + l;
        return math.mul(R(theta1), new float2(iprimex, 0)) + p1;
    }

    static float2x2 R(float theta) {
        return new float2x2(math.cos(theta), -math.sin(theta), math.sin(theta), math.cos(theta));
    }

    public static Vector2 rotate(Vector2 v, float delta) {
        return new Vector2(
            v.x * Mathf.Cos(delta) - v.y * Mathf.Sin(delta),
            v.x * Mathf.Sin(delta) + v.y * Mathf.Cos(delta)
        );
    }



    float drawRadius = .5f;
    bool MouseMove(ref Vector2 point) {
        Vector2 worldPosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);
        if (Input.GetMouseButton(0)) {
            if ((point - worldPosition).magnitude < drawRadius) {
                point = worldPosition;
                return true;
            }
        }
        return false;
    }
    
}

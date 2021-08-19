using System.Collections.Generic;

using Unity.Entities;
using Unity.Mathematics;

using Random = UnityEngine.Random;

using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEditor;

using Physics.Math;

public class LightAuthoring : MonoBehaviour, IConvertGameObjectToEntity {

    public float angularVelocity;
    public float inertia = .1f;
    public float aperture = 40;

    public float snapRadius = 1;

    public GameObject toTrack;
    public float trackSpeed;
    public float trackSoftness;

    [System.Serializable]
    public struct SnapInfo {
        public Vector2 pos;
        public Vector2 ccw;
        public Vector2 cw;
        public GameObject mount;

        public void Set(SerializedProperty prop) {
            prop.FindPropertyRelative("pos").vector2Value = pos;
            prop.FindPropertyRelative("ccw").vector2Value = ccw;
            prop.FindPropertyRelative("cw").vector2Value = cw;
            prop.FindPropertyRelative("mount").objectReferenceValue = mount;
        }
    }

    public SnapInfo[] snapInfos = new SnapInfo[0];

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) {
        dstManager.AddComponentData(entity, GetLightSource());
        dstManager.AddComponentData(entity, GetLightPosition());

        dstManager.AddComponentData(entity, 
            new Velocity {
                vel = 0,
                angVel = angularVelocity*Mathf.Deg2Rad,
            }
        );
        dstManager.AddComponentData(entity, 
            new Mass {
                mass = math.INFINITY,
                inertia = inertia
            }
        );

        dstManager.AddComponentData(entity,
            new GravityScale {gravityScale = 0}
        );

        dstManager.AddComponentData(entity, new IgnoreHierarchyTag());

        if (toTrack != null) {
            dstManager.AddComponentData(entity,
                new LightTrackingJoint {
                    toTrack = conversionSystem.GetPrimaryEntity(toTrack),
                    trackSpeed = math.radians(trackSpeed),
                    trackSoftness = trackSoftness
                }
            );
        }

        foreach (var snapInfo in snapInfos) {
            var joint = conversionSystem.CreateAdditionalEntity(gameObject);
            dstManager.SetName(joint, "Light mount joint");
            dstManager.AddComponentData(joint,
                new LightMountJoint {
                    lightEntity = entity,
                    mount = conversionSystem.GetPrimaryEntity(snapInfo.mount),
                    rangeStart = Ang.SignedAngleOf(snapInfo.ccw),
                    rangeEnd = Ang.SignedAngleOf(snapInfo.cw),
                    id = Random.Range(0, int.MaxValue)
                }
            );
        }
    }

    public Position GetLightPosition() {
        var snapPosition = Vector2.zero;

        if (snapInfos == null || snapInfos.Length == 0) {
            snapPosition = transform.position;
        } else {
            foreach (var info in snapInfos) {
                snapPosition += (Vector2)info.mount.transform.TransformPoint(info.pos);
            }
            snapPosition /= snapInfos.Length;
        }

        return new Position {
            pos = snapPosition,
            rot = transform.eulerAngles.z*Mathf.Deg2Rad,
        };
    }

    public LightSource GetLightSource() {
        return new LightSource {
                aperture = aperture * Mathf.Deg2Rad,
                maxEdgeId = Random.Range(1, int.MaxValue),
                minEdgeId = Random.Range(1, int.MaxValue),
                id = Random.Range(1, int.MaxValue),
            };
    }

    void OnDrawGizmos() {
        var posComponent = GetLightPosition();
        Vector2 pos = posComponent.pos;
        float rot = math.degrees(posComponent.rot);

        Gizmos.color = Color.red;
        Gizmos.DrawSphere(pos, .1f);

        Gizmos.DrawRay(pos, Quaternion.Euler(0,0,rot+aperture/2)*Vector2.right*20);
        Gizmos.DrawRay(pos, Quaternion.Euler(0,0,rot-aperture/2)*Vector2.right*20);

        foreach (var snapInfo in snapInfos) {
            Gizmos.DrawRay(pos, snapInfo.mount.transform.TransformDirection(snapInfo.cw));
            Gizmos.DrawRay(pos, snapInfo.mount.transform.TransformDirection(snapInfo.ccw));
        }

        if (toTrack != null) {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(transform.position, toTrack.transform.position);
        }
    }

    static SnapInfo[] GetSnapInfo(Transform transform, float snapRadius) {
        var snapInfos = new List<SnapInfo>();
        void Search(Transform t) {
            foreach (var child in t.GetComponentsInChildren<BoxAuthoring>()) {
                var closestPoint = child.GetRect().ClosestPoint((Vector2)transform.position, ccwVec: out var ccwVec, cwVec: out var cwVec);
                if (math.distance(closestPoint, (Vector2)transform.position) < snapRadius) {
                    Vector2 WorldToLocalVec(float2 vec) {
                        return child.transform.InverseTransformDirection((Vector2)math.normalize(vec));
                    }

                    ccwVec = WorldToLocalVec(ccwVec);
                    cwVec = WorldToLocalVec(cwVec);

                    snapInfos.Add(new SnapInfo{
                        pos = (Vector2)child.transform.InverseTransformPoint((Vector2)closestPoint) + (Vector2)math.normalize(Lin.Cross(ccwVec, 1))*.1f,
                        ccw = ccwVec,
                        cw = cwVec,
                        mount = child.gameObject
                    });
                }
            }
        }

        if (transform.parent == null) {
            foreach (var root in SceneManager.GetActiveScene().GetRootGameObjects()) {
                Search(root.transform);
            }
        } else {
            Search(transform.parent);
        }

        return snapInfos.ToArray();
    }

    [CustomEditor(typeof(LightAuthoring))]
    private class LightEditor : Editor {
        SerializedProperty snapInfosField;

        void OnEnable() {
            snapInfosField = serializedObject.FindProperty("snapInfos");
        }

        public override void OnInspectorGUI() {
            void DrawProperty(string propName) {
                EditorGUILayout.PropertyField(serializedObject.FindProperty(propName), true);
            }

            serializedObject.Update();

            var t = target as LightAuthoring;

            DrawProperty("angularVelocity");
            DrawProperty("inertia");
            DrawProperty("aperture");
            DrawProperty("toTrack");
            if (t.toTrack != null) {
                DrawProperty("trackSpeed");
                DrawProperty("trackSoftness");
            }
            GUILayout.Label("");
            DrawProperty("snapRadius");

            serializedObject.ApplyModifiedProperties();

            GUILayout.Label("");

            serializedObject.Update();


            if (GUI.changed || t.transform.hasChanged) {
                snapInfosField.ClearArray();
                foreach(var snapInfo in LightAuthoring.GetSnapInfo(t.transform, t.snapRadius)) {
                    snapInfosField.InsertArrayElementAtIndex(0);
                    snapInfo.Set(snapInfosField.GetArrayElementAtIndex(0));
                }
            }

            bool wasEnabled = GUI.enabled;
            GUI.enabled = false;

            DrawProperty("snapInfos");

            GUI.enabled = wasEnabled;

            serializedObject.ApplyModifiedPropertiesWithoutUndo();
        }
    }
}

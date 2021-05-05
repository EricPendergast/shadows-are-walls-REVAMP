using System.Collections.Generic;
using UnityEngine;

[ExecuteAlways]
public class EditorLightRenderer : MonoBehaviour {
    void Update() {
        if (!Application.isPlaying) {
            ShadowRenderPassFeature.lights.Clear();
            
            foreach (LightAuthoring l in Object.FindObjectsOfType<LightAuthoring>()) {
                var lightSource = l.GetLightSource();
                ShadowRenderPassFeature.lights.Add(lightSource.GetLightMatrix());
            }
        }
    }

    void OnDisable() {
        if (!Application.isPlaying) {
            ShadowRenderPassFeature.lights.Clear();
        }
    }
}

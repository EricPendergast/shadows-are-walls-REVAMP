using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class EditorLightRenderer : MonoBehaviour {
    void Update() {
        List<Matrix4x4> lights = new List<Matrix4x4>(50);
        
        foreach (LightAuthoring l in Object.FindObjectsOfType<LightAuthoring>()) {
            var lightSource = l.GetLightSource();
            lights.Add(lightSource.GetLightMatrix());
        }

        int lightsCount = lights.Count;
        // Since you can't resize a matrix array, we need to allocate the max amount right away.
        for (int i = lightsCount; i < 50; i++) {
            lights.Add(Matrix4x4.zero);
        }

        Shader.SetGlobalMatrixArray(GlobalShaderProperties.lights, lights);
        Shader.SetGlobalInt(GlobalShaderProperties.numLights, lightsCount);
    }

    void OnDisable() {
        Shader.SetGlobalInt(GlobalShaderProperties.numLights, 0);
    }
}

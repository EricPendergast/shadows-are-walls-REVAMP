using UnityEngine;

[CreateAssetMenu(fileName = "ShaderReferences", menuName = "ScriptableObjects/ShaderReferences")]
public class ShaderReferences : SingletonScriptableObject<ShaderReferences> {
    public Shader shadowStencilShader;
    public RenderTexture shadowStencilRenderTexture;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    static void Init() {
        SingletonInit();
    }
}

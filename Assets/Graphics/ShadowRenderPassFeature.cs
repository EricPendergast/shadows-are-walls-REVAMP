using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

public class ShadowRenderPassFeature : ScriptableRendererFeature {
    public Material lightConeMaterial;

    class CustomRenderPass : ScriptableRenderPass {
        public readonly int numLightsId = Shader.PropertyToID(GlobalShaderProperties.numLights);
        public readonly int currentLightId = Shader.PropertyToID(GlobalShaderProperties.currentLight);

        public Material lightConeMaterial;

        public CustomRenderPass() {
            // In case this is uninitialized
            Shader.SetGlobalInt(numLightsId, 0);
        }

        // This method is called before executing the render pass.
        // It can be used to configure render targets and their clear state. Also to create temporary render target textures.
        // When empty this render pass will render to the active camera render target.
        // You should never call CommandBuffer.SetRenderTarget. Instead call <c>ConfigureTarget</c> and <c>ConfigureClear</c>.
        // The render pipeline will ensure target setup and clearing happens in a performant manner.
        public override void OnCameraSetup(CommandBuffer cmd, ref RenderingData renderingData) {
            ConfigureClear(ClearFlag.Depth, Color.clear);
        }

        // Here you can implement the rendering logic.
        // Use <c>ScriptableRenderContext</c> to issue drawing commands or execute command buffers
        // https://docs.unity3d.com/ScriptReference/Rendering.ScriptableRenderContext.html
        // You don't have to call ScriptableRenderContext.submit, the render pipeline will call it at specific points in the pipeline.
        public override void Execute(ScriptableRenderContext context, ref RenderingData renderingData) {

            var camera = renderingData.cameraData.camera;
            var prevCullingMatrix = camera.cullingMatrix;
            camera.cullingMatrix = Matrix4x4.Ortho(float.MinValue, float.MaxValue, float.MinValue, float.MaxValue, float.MinValue, float.MaxValue);
            if (!camera.TryGetCullingParameters(out var cullingParameters)) {
                return;
            }
            camera.cullingMatrix = prevCullingMatrix;
            CommandBuffer cmd = new CommandBuffer(){
                name = "Render Light"
            };

            // ShaderTagId corresponds to putting the following in a shader pass:
            //      Tags { "LightMode" = "ShadowDrawPass" }
            var drawingSettings = new DrawingSettings(
                new ShaderTagId("ShadowDrawPass"), 
                new SortingSettings(renderingData.cameraData.camera)) {
            };
            var filteringSettings = FilteringSettings.defaultValue;

            var cullingResults = context.Cull(ref cullingParameters);

            int numLights = Shader.GetGlobalInt(numLightsId);
            
            for (int i = 0; i < numLights; i++) {
                cmd.SetGlobalInt(currentLightId, i);
                context.ExecuteCommandBuffer(cmd);
                cmd.Clear();
                context.DrawRenderers(
                    cullingResults, ref drawingSettings, ref filteringSettings);
                cmd.DrawProcedural(
                        Matrix4x4.identity, lightConeMaterial, 0,
                        MeshTopology.Triangles, 3);
                cmd.ClearRenderTarget(true, false, Color.clear, 1);
                context.ExecuteCommandBuffer(cmd);
                cmd.Clear();
            }
        }

        // Cleanup any allocated resources that were created during the execution of this render pass.
        public override void OnCameraCleanup(CommandBuffer cmd) {
        }
    }

    CustomRenderPass m_ScriptablePass;

    public override void Create() {
        if (this.lightConeMaterial == null) {

            Debug.LogWarning("Warning: Field(s) not initialized in ShadowRenderPassFeature. Not using this pass.");

            m_ScriptablePass = null;
            return;
        }

        m_ScriptablePass = new CustomRenderPass();

        m_ScriptablePass.lightConeMaterial = lightConeMaterial;
        
        m_ScriptablePass.renderPassEvent = RenderPassEvent.BeforeRenderingOpaques;
    }

    // Here you can inject one or multiple render passes in the renderer.
    // This method is called when setting up the renderer once per-camera.
    public override void AddRenderPasses(ScriptableRenderer renderer, ref RenderingData renderingData) {
        if (m_ScriptablePass != null) {
            renderer.EnqueuePass(m_ScriptablePass);
        }
    }
}



using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

using Unity.Entities;
using Unity.Collections;

public class ShadowRenderPassFeature : ScriptableRendererFeature {
    public Shader shadowStencilShader;
    public Shader shadowStencilBlitShader;
    public Material lightConeMaterial;

    class CustomRenderPass : ScriptableRenderPass {
        public Material shadowStencilMaterial;
        public Material shadowStencilBlitMaterial;
        public Material lightConeMaterial;

        //int shadowStencilId =  Random.Range(0, int.MaxValue);
        // This method is called before executing the render pass.
        // It can be used to configure render targets and their clear state. Also to create temporary render target textures.
        // When empty this render pass will render to the active camera render target.
        // You should never call CommandBuffer.SetRenderTarget. Instead call <c>ConfigureTarget</c> and <c>ConfigureClear</c>.
        // The render pipeline will ensure target setup and clearing happens in a performant manner.
        public override void OnCameraSetup(CommandBuffer cmd, ref RenderingData renderingData) {
            //cmd.GetTemporaryRT(shadowStencilId, renderingData.cameraData.camera.pixelWidth, renderingData.cameraData.camera.pixelHeight, 32, FilterMode.Point, RenderTextureFormat.Depth);
            //ConfigureTarget(renderingData.cameraData.targetTexture, shadowStencilId);
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

            //var world = World.DefaultGameObjectInjectionWorld;
            //
            //var lightSourcesQuery = world.EntityManager.CreateEntityQuery(typeof(LightSource));
            //NativeArray<LightSource> lightSources = lightSourcesQuery.ToComponentDataArray<LightSource>(Allocator.Temp);
            //
            //var boxesQuery = world.EntityManager.CreateEntityQuery(typeof(Box));
            //NativeArray<Box> boxes = boxesQuery.ToComponentDataArray<Box>(Allocator.Temp);

            var drawingSettings = new DrawingSettings(
                new ShaderTagId("OpaqueObjectPass"), 
                new SortingSettings(renderingData.cameraData.camera)) {
                //overrideMaterial = shadowStencilMaterial
            };
            var filteringSettings = FilteringSettings.defaultValue;

            // TODO: Figure out how to turn off culling completely
            //cullingParameters = new ScriptableCullingParameters(){
            //    cullingMask = cullingParameters.cullingMask
            //};
            //cullingParameters.cullingOptions = CullingOptions.DisablePerObjectCulling;
            //cullingParameters.cullingMatrix = Matrix4x4.Ortho(float.MinValue, float.MaxValue, float.MinValue, float.MaxValue, float.MinValue, float.MaxValue);
            //cullingParameters.cullingMatrix = Matrix4x4.identity;
            //cullingParameters.isOrthographic = true;

            //for (int i = 0; i < 5; i++) {
            //    cullingParameters.cameraProperties.SetCameraCullingPlane(i, new Plane(new Vector3(1, 0, 0), 100000));
            //}
            var cullingResults = context.Cull(ref cullingParameters);

            //for (int i = 0; i < lightSources.Length; i++) {
                context.DrawRenderers(
                    cullingResults, ref drawingSettings, ref filteringSettings);
            //}

            cmd.DrawProcedural(
                    Matrix4x4.identity, lightConeMaterial, 0,
                    MeshTopology.Triangles, 3);
            //Blit(cmd, shadowStencilId, camera.activeTexture);

            context.ExecuteCommandBuffer(cmd);

            //lightSources.Dispose();
            //boxes.Dispose();
        }

        // Cleanup any allocated resources that were created during the execution of this render pass.
        public override void OnCameraCleanup(CommandBuffer cmd) {
            //cmd.ReleaseTemporaryRT(shadowStencilId);
        }
    }

    CustomRenderPass m_ScriptablePass;

    /// <inheritdoc/>
    public override void Create() {
        if ( this.shadowStencilShader == null ||
                this.shadowStencilBlitShader == null ||
                this.lightConeMaterial == null) {
            Debug.LogWarning("Warning: Field(s) not initialized in ShadowRenderPassFeature. Not using this pass.");

            m_ScriptablePass = null;
            return;
        }

        m_ScriptablePass = new CustomRenderPass();

        m_ScriptablePass.shadowStencilMaterial = new Material(shadowStencilShader);
        m_ScriptablePass.shadowStencilBlitMaterial = new Material(shadowStencilBlitShader);
        m_ScriptablePass.lightConeMaterial = lightConeMaterial;
        
        // Configures where the render pass should be injected.
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



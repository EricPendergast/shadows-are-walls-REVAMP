Shader "Custom RP/LightCone" {
    Properties {
        _Color ("Color", Color) = (1,0,0,1)
    }
    SubShader {
        Cull Off
        ZTest LEqual
        /*ZWrite Off*/
        // This needs to be here because without it, in the build there are
        // strange artifacts around the borders of triangles.
        Blend SrcAlpha OneMinusSrcAlpha

        HLSLINCLUDE
        #include "../ShaderLibrary/Common.hlsl"
        ENDHLSL

        Pass {
            // Only render the fragment if stencil is zero.
            /*Stencil {*/
            /*    Ref 0*/
            /*    Comp Equal*/
            /*    Pass keep*/
            /*}*/
            HLSLPROGRAM
                #pragma target 3.5
                #pragma vertex ProceduralVertex
                #pragma fragment Fragment

                float4 _Color;

                float4 ProceduralVertex(uint vertexID : SV_VertexID) : POSITION {
                    return float4(
                        vertexID <= 1 ? -1.0 : 3.0,
                        vertexID == 1 ? 3.0 : -1.0,
                        0.0, 1.0
                    );
                }
                /*float4 Vertex(float3 vertex : POSITION) : POSITION {*/
                /*    return TransformObjectToHClip(vertex);*/
                /*}*/

                float4 Fragment(float4 position : POSITION) : SV_TARGET {
                    return float4(_Color.rgb, 1);
                }
            ENDHLSL
        }
    }
}

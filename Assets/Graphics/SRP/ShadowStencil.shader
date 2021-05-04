Shader "Unlit/ShadowStencil" {
    Properties {
        _Color ("Color", Color) = (1,0,1,1)
        _WriteMask ("WriteMask", int) = 1
    }
    SubShader {
        /*Stencil {*/
        /*    Ref 0*/
        /*    Comp Always*/
        /*    WriteMask [_WriteMask]*/
        /*}*/
        Cull Off
        ZTest LEqual
        ZWrite On
        // This needs to be here because without it, in the build there are
        // strange artifacts around the borders of triangles.
        Blend SrcAlpha OneMinusSrcAlpha

        HLSLINCLUDE
        #include "../ShaderLibrary/Common.hlsl"
        ENDHLSL

        Tags { "LightMode" = "SRPDefaultUnlit" }

        Pass {
            HLSLPROGRAM
                #pragma target 3.5
                #pragma vertex Vertex
                #pragma fragment Fragment

                float4 _Color;

                float4 Vertex(float3 vertex : POSITION) : POSITION {
                    return TransformObjectToHClip(vertex);
                }

                float4 Fragment(float4 position : POSITION) : SV_TARGET {
                    return float4(_Color.rgb, 1);
                }
            ENDHLSL
        }
    }
}

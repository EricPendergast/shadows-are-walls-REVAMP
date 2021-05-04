Shader "Custom RP/Transparent Unlit" {
    Properties {
        _Color ("Color", Color) = (1,0,1,1)
    }
    SubShader {
        Cull Off
        ZTest LEqual
        ZWrite Off
        Blend SrcAlpha OneMinusSrcAlpha
        Tags { "Queue" = "Transparent" }
        

        HLSLINCLUDE
        #include "../ShaderLibrary/Common.hlsl"
        ENDHLSL

        Pass {
            Tags { "LightMode" = "SRPDefaultUnlit" }
            HLSLPROGRAM
                #pragma target 3.5
                #pragma vertex Vertex
                #pragma fragment Fragment

                float4 _Color;

                float4 Vertex(float3 vertex : POSITION) : POSITION {
                    return TransformObjectToHClip(vertex);
                }

                float4 Fragment(float4 position : POSITION) : SV_TARGET {
                    return _Color;
                }
            ENDHLSL
        }
    }
}

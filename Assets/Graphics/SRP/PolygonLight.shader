Shader "Custom RP/Polygon Light" {
    Properties {
        _Color ("Color", Color) = (1,0,1,1)
    }
    SubShader {
        Cull Off
        ZTest LEqual
        ZWrite Off
        Blend SrcAlpha OneMinusSrcAlpha

        HLSLINCLUDE
        #include "../ShaderLibrary/Common.hlsl"
        #include "PolygonLight.hlsl"
        ENDHLSL

        Pass {
            Tags { "LightMode" = "SRPDefaultUnlit" }
            HLSLPROGRAM
                #pragma target 3.5
                #pragma vertex Vertex
                #pragma fragment Fragment
            ENDHLSL
        }
    }
}

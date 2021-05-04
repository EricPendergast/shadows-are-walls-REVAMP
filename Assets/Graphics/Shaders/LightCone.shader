Shader "Custom RP/LightCone" {
    Properties {
        _Color ("Color", Color) = (1,0,0,1)
    }
    SubShader {
        Cull Off
        ZTest LEqual
        ZWrite Off
        // This needs to be here because without it, in the build there are
        // strange artifacts around the borders of triangles.
        Blend SrcAlpha OneMinusSrcAlpha

        HLSLINCLUDE
        #include "../ShaderLibrary/Common.hlsl"
        ENDHLSL

        Pass {
            // Only render the fragment if stencil is zero.
            Stencil {
                Ref 0
                Comp Equal
                Pass keep
            }
            HLSLPROGRAM
                #pragma target 3.5
                #pragma vertex ProceduralVertex
                #pragma fragment Fragment

                float4 _Color;
                float4x4 lights[50];
                int numLights;
                int currentLight;

                float4 ProceduralVertex(uint vertexID : SV_VertexID) : POSITION {
                    float4x4 l = lights[currentLight];
                    float4 worldPoint = float4(0, 0, 0, 1);
                    float2 lightPos = l._m00_m10;
                    if (vertexID == 0) {
                        worldPoint = float4(lightPos, 0, 1);
                    } else if (vertexID == 1) {
                        worldPoint = float4(lightPos + l._m01_m11*300, 0, 1);
                    } else if (vertexID == 2) {
                        worldPoint = float4(lightPos + l._m02_m12*300, 0, 1);
                    }
                    return TransformWorldToHClip(worldPoint);
                }

                float4 Fragment(float4 position : POSITION) : SV_TARGET {
                    return _Color;
                }
            ENDHLSL
        }
    }
}

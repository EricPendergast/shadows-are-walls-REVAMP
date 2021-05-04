Shader "Custom RP/Opaque" {
    Properties {
        _Color ("Color", Color) = (1,0,0,1)
    }
    SubShader {
        Cull Off

        HLSLINCLUDE
        #include "../ShaderLibrary/Common.hlsl"
        ENDHLSL

        Pass {
            ZTest LEqual
            ZWrite On
            // This needs to be here because without it, in the build there are
            // strange artifacts around the borders of triangles.
            Blend SrcAlpha OneMinusSrcAlpha
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
                    return float4(_Color.rgb, 1);
                }
            ENDHLSL
        }

        Pass {
            ZWrite On
            ZTest Always
            Blend Off
            ColorMask 0
            
            Tags { "LightMode" = "ShadowDrawPass" }
            // Write 1 to the stencil buffer for each fragment. Don't
            // render any fragment.
            Stencil {
                Ref 1
                Pass Replace
                Comp Always
            }
            HLSLPROGRAM
                #pragma target 3.5
                #pragma require geometry
                #pragma vertex Vertex
                #pragma fragment Fragment
                #pragma geometry geom

                float4 _Color;
                float4x4 lights[50];
                int numLights;
                int currentLight;

                struct v2g {
                    float4 vertex : POSITION;
                };
                
                struct g2f {
                    float4 vertex : SV_POSITION;
                };

                v2g Vertex(float3 vertex : POSITION) {
                    v2g o;
                    o.vertex = float4(TransformObjectToWorld(vertex), 1);

                    return o;
                }

                // TODO: This can be with maxvertexcount(2)
                [maxvertexcount(18)]
                void geom(triangle v2g input[3], inout TriangleStream<g2f> outputStream) {

                    float2 lightPos = lights[currentLight]._m00_m10;

                    for (int i = 0; i < 3; i++) {
                        g2f o1;
                        g2f o2;
                        g2f o3;
                        g2f o4;
                        o1.vertex = input[i].vertex;
                        o2.vertex = input[(i + 1)%3].vertex;
                        o3.vertex = float4(lightPos + normalize(o1.vertex.xy - lightPos)*200, 0, 1);
                        o4.vertex = float4(lightPos + normalize(o2.vertex.xy - lightPos)*200, 0, 1);

                        o1.vertex = TransformWorldToHClip(o1.vertex);
                        o2.vertex = TransformWorldToHClip(o2.vertex);
                        o3.vertex = TransformWorldToHClip(o3.vertex);
                        o4.vertex = TransformWorldToHClip(o4.vertex);

                        outputStream.Append(o1);
                        outputStream.Append(o2);
                        outputStream.Append(o4);
                        outputStream.RestartStrip();
                        outputStream.Append(o1);
                        outputStream.Append(o3);
                        outputStream.Append(o4);
                        outputStream.RestartStrip();
                    }
                }

                void Fragment(g2f input) {}
            ENDHLSL
        }
    }
}

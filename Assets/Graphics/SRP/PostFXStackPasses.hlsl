#ifndef CUSTOM_POST_FX_PASSES_INCLUDED
#define CUSTOM_POST_FX_PASSES_INCLUDED

struct Varyings {
    float4 positionCS : SV_POSITION;
    float2 fxUV : VAR_FX_UV;
};

Varyings DefaultPassVertex(uint vertexID : SV_VertexID) {
    Varyings output;
    output.positionCS = float4(
        vertexID <= 1 ? -1.0 : 3.0,
        vertexID == 1 ? 3.0 : -1.0,
        0.0, 1.0
    );

    output.fxUV = float2(
        vertexID <= 1 ? 0.0 : 2.0,
        vertexID == 1 ? 2.0 : 0.0
    );

    if (_ProjectionParams.x < 0.0) {
        output.fxUV.y = 1.0 - output.fxUV.y;
    }

    return output;
}

TEXTURE2D(_PostFXSource);
SAMPLER(sample_linear_clamp);

float4 GetSource(float2 fxUV) {
    return SAMPLE_TEXTURE2D(_PostFXSource, sample_linear_clamp, fxUV);
}


float4 CopyPassFragment(Varyings input) : SV_TARGET {
    return GetSource(input.fxUV);
}


float ApplyCCCurve(float channel) {
    return pow((1-1/(6*channel+1)), 2);
}

float4 ColorCorrectFragment(Varyings input) : SV_TARGET {
    // TODO: This probably isn't very good color correction
    float4 color = GetSource(input.fxUV);
    return float4(
        ApplyCCCurve(color.r),
        ApplyCCCurve(color.g),
        ApplyCCCurve(color.b),
        color.a);
}

float4 GrayscaleFragment(Varyings input) : SV_TARGET {
    float4 color = GetSource(input.fxUV);
    float intensity = (color.r + color.g + color.b)/3;
    return float4(intensity, intensity, intensity, color.a);
}

#endif

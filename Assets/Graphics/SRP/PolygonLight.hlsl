#ifndef POLYGON_LIGHT_INCLUDED
#define POLYGON_LIGHT_INCLUDED

struct Varyings {
    float4 vertex : POSITION;
};

struct FragOut {
    float4 color : SV_TARGET;
};

float4 _Color;

Varyings Vertex(float3 vertex : POSITION) {
    Varyings output;
    output.vertex = TransformObjectToHClip(vertex);
    return output;
}

FragOut Fragment(Varyings input) {
    FragOut output;
    output.color = _Color;
    return output;
}

#endif

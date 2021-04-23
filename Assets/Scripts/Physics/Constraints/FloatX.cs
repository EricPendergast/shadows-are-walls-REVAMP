using Unity.Mathematics;

public interface FloatX<T> {
    public T Add(T other);
    public float Dot(T other);
    public T Mult(T other);
    public T Mult(float other);
}

public struct Float6 : FloatX<Float6> {
    public float3 v1;
    public float3 v2;

    public Float6(float3 v1, float3 v2) {
        this.v1 = v1;
        this.v2 = v2;
    }

    public Float6(float f1, float f2, float f3, float f4, float f5, float f6) {
        this.v1 = new float3(f1, f2, f3);
        this.v2 = new float3(f4, f5, f6);
    }

    public Float6 Add(Float6 o) {
        return new Float6(v1 + o.v1, v2 + o.v2);
    }

    public float Dot(Float6 o) {
        return math.dot(v1, o.v1) + math.dot(v2, o.v2);
    }

    public Float6 Mult(Float6 o) {
        return new Float6(v1*o.v1, v2*o.v2);
    }

    public Float6 Mult(float o) {
        return new Float6(v1*o, v2*o);
    }
}

public struct Float4 : FloatX<Float4> {
    public float4 v;

    public static implicit operator float4(Float4 v) {
        return v;
    }
    public static implicit operator Float4(float4 v) {
        return new Float4(v);
    }
    public Float4(float4 v) {
        this.v = v;
    }

    public Float4(float f1, float f2, float f3, float f4) {
        this.v = new float4(f1, f2, f3, f4);
    }

    public Float4(float2 f12, float f3, float f4) {
        this.v = new float4(f12, f3, f4);
    }

    public Float4 Add(Float4 o) {
        return v + o.v;
    }

    public float Dot(Float4 o) {
        return math.dot(this, o);
    }

    public Float4 Mult(Float4 o) {
        return v * o.v;
    }

    public Float4 Mult(float o) {
        return v * o;
    }
}


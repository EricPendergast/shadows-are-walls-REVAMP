using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

using Physics.Math;

using ContactId = Physics.Math.Geometry.ContactId;

// A constraint that restricts two degrees of freedom
public readonly struct TwoDOFConstraint<T> where T : FloatX<T> {
    public readonly T J1;
    public readonly T J2;

    public readonly float2x2 m_c;

    public readonly float2 bias;

    public readonly float softness;

    public TwoDOFConstraint(T J1, T J2, T M_inv, float2 bias, float softness) {
        this.J1 = J1;
        this.J2 = J2;

        this.bias = bias;
        this.softness = softness;

        float2x2 K = new float2x2();

        K.c0.x = J1.Mult(M_inv).Dot(J1);    K.c1.x = J1.Mult(M_inv).Dot(J2);
        K.c0.y = J2.Mult(M_inv).Dot(J1);    K.c1.y = J2.Mult(M_inv).Dot(J2);

        K.c0.x += softness;
        K.c1.y += softness;


        m_c = math.inverse(K);
        //m_c = new float2(1/(J1.Mult(M_inv).Dot(J1) + softness), 1/(J2.Mult(M_inv).Dot(J2) + softness));
    }

    private TwoDOFConstraint(T J1, T J2, float2x2 m_c, float2 bias, float softness) {
        this.J1 = J1;
        this.J2 = J2;
        this.m_c = m_c;
        this.bias = bias;
        this.softness = softness;
    }

    public TwoDOFConstraint<T> WithBiasMultiplied(float biasMult) {
        return new TwoDOFConstraint<T>(J1: J1, J2: J2, m_c: m_c, bias: bias*biasMult, softness: softness);
    }

    public T GetImpulse(float2 lambda) {
        return J1.Mult(lambda.x).Add(J2.Mult(lambda.y));
    }

    public T GetImpulse(T v, ref float2 accumulatedLambda) {
        float2 thing = new float2(J1.Dot(v), J2.Dot(v)) + softness*accumulatedLambda + bias;
        float2 lambda = math.mul(
            -m_c,
            thing
        );
        //float2 lambda = new float2(
        //    -m_c.x * (J1.Dot(v) + softness*accumulatedLambda.x + bias.x),
        //    -m_c.y * (J2.Dot(v) + softness*accumulatedLambda.y + bias.y)
        //);
        accumulatedLambda += lambda;

        return GetImpulse(lambda);
    }

}

public readonly struct OneDOFConstraint<T> where T : FloatX<T> {
    public readonly T J;

    public readonly float m_c;

    public readonly float bias;

    public readonly float softness;

    public OneDOFConstraint(T J, T M_inv, float bias, float softness) {
        this.J = J;
    
        this.bias = bias;
        this.softness = softness;
    
        m_c = 1 / (J.Mult(M_inv).Dot(J) + softness);
    }

    public T GetImpulse(float lambda) {
        return J.Mult(lambda);
    }
    
    public T GetImpulse(T v, ref float accumulatedLambda) {
        float lambda = GetLambda(v, ref accumulatedLambda);
    
        return J.Mult(lambda);
    }
    
    public float GetLambda(T v, ref float accumulatedLambda) {
        float lambda = -m_c * (J.Dot(v) + softness*accumulatedLambda + bias);
        accumulatedLambda += lambda;
        return lambda;
    }
}

public readonly struct PenetrationConstraint<T> where T : FloatX<T> {
    public readonly T J;
    public readonly float m_c;

    public readonly float bias;

    public PenetrationConstraint(T J, T M_inv, float bias) {
        this.J = J;
    
        this.bias = bias;
    
        m_c = 1 / (J.Mult(M_inv).Dot(J) + CollisionSystem.globalSoftness);
    }

    private PenetrationConstraint(T J, float m_c, float bias) {
        this.J = J;
        this.m_c = m_c;
        this.bias = bias;
    }

    public PenetrationConstraint<T> WithBiasMultiplied(float biasMult) {
        return new PenetrationConstraint<T>(J: this.J, m_c: this.m_c, bias: this.bias*biasMult);
    }

    public T GetImpulse(float lambda) {
        return J.Mult(lambda);
    }
    
    public T GetImpulse(T v, ref float accumulatedLambda) {
        float lambda = GetLambda(v, ref accumulatedLambda);
    
        return J.Mult(lambda);
    }
    
    public float GetLambda(T v, ref float accumulatedLambda) {
        float lambda = -m_c * (J.Dot(v) + CollisionSystem.globalSoftness*accumulatedLambda + bias);
        //accumulatedLambda += lambda;
        ClampLambda(ref lambda, ref accumulatedLambda);
        return lambda;
    }
    
    public static void ClampLambda(ref float lambda, ref float accumulated) {
        if (CollisionSystem.accumulateImpulses) {
            float oldAccumulated = accumulated;
            accumulated = math.max(accumulated + lambda, 0);
            lambda = accumulated - oldAccumulated;
        } else {
            lambda = math.max(lambda, 0);
        }
    }
}

public readonly struct FrictionConstraint<T> where T : FloatX<T> {

    public readonly T J;
    public readonly float m_c;
    public readonly float frictionCoeff;

    public FrictionConstraint(T J, T M_inv, float frictionCoeff) {
        this.J = J;
        m_c = 1 / (J.Mult(M_inv).Dot(J));

        this.frictionCoeff = frictionCoeff;
    }

    public T GetImpulse(float lambda) {
        return J.Mult(lambda);
    }
    
    public T GetImpulse(T v, ref float accum_t, ref float accum_n) {
        float lambda_t = -m_c * (J.Dot(v));

        ClampLambda(ref lambda_t, ref accum_t, ref accum_n);

        return J.Mult(lambda_t);
    }
    
    private void ClampLambda(ref float lambda_t, ref float accum_t, ref float accum_n) {
        float old_tAccumulated = accum_t;
        accum_t = math.clamp(accum_t + lambda_t, -accum_n*frictionCoeff, accum_n*frictionCoeff);
        lambda_t = accum_t - old_tAccumulated;

        // This is what we would do if we didn't use accumulation:
        // lambda_t = math.clamp(lambda_t, -lambda_n*frictionCoeff, lambda_n*frictionCoeff);
    }
}

using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

using Physics.Math;

using ContactId = Physics.Math.Geometry.ContactId;

public readonly struct PenetrationConstraint<T> where T : FloatX<T> {
    public readonly T J;
    public readonly float m_c;

    public readonly float bias;

    public PenetrationConstraint(T J, T M_inv, float bias) {
        this.J = J;
    
        this.bias = bias;
    
        m_c = 1 / (J.Mult(M_inv).Dot(J));
    }

    public T GetImpulse(float lambda) {
        return J.Mult(lambda);
    }
    
    public T GetImpulse(T v, ref float accumulatedLambda) {
        float lambda = GetLambda(v, ref accumulatedLambda);
    
        return J.Mult(lambda);
    }
    
    public float GetLambda(T v, ref float accumulatedLambda) {
        float lambda = -m_c * (J.Dot(v) + bias);
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

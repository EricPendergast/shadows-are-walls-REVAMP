using Unity.Entities;
using Unity.Mathematics;

using Physics.Math;

using ContactId = Physics.Math.Geometry.ContactId;

public readonly struct PenetrationConstraint {
    public readonly float3 J1;
    public readonly float3 J2;
    public readonly float m_c;

    public readonly float bias;

    public PenetrationConstraint(float3 J1, float3 J2, float3 M1_inv, float3 M2_inv, float bias) {
        this.J1 = J1;
        this.J2 = J2;
    
        this.bias = bias;
    
        m_c = 1 / (math.dot(J1 * M1_inv, J1) + math.dot(J2 * M2_inv, J2));
    }

    public (float3, float3) GetImpulse(float lambda) {
        return (J1*lambda, J2*lambda);
    }
    
    public (float3, float3) GetImpulse(float3 v1, float3 v2, ref float accumulatedLambda) {
        float lambda = GetLambda(v1, v2, ref accumulatedLambda);
    
        return (J1*lambda, J2*lambda);
    }
    
    public float GetLambda(float3 v1, float3 v2, ref float accumulatedLambda) {
        float lambda = -m_c * (math.dot(J1, v1) + math.dot(J2, v2) + bias);
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



public readonly struct FrictionConstraint {
    public readonly float3 J1;
    public readonly float3 J2;
    public readonly float m_c;
    public readonly float frictionCoeff;

    public FrictionConstraint(float3 J1, float3 J2, float3 M1_inv, float3 M2_inv, float frictionCoeff) {
        this.J1 = J1;
        this.J2 = J2;
    
        m_c = 1 / (math.dot(J1 * M1_inv, J1) + math.dot(J2 * M2_inv, J2));

        this.frictionCoeff = frictionCoeff;
    }

    public (float3, float3) GetImpulse(float lambda) {
        return (J1*lambda, J2*lambda);
    }
    
    public (float3, float3) GetImpulse(float3 v1, float3 v2, float lambda_n, ref float accum_t, ref float accum_n) {
        float lambda_t = -m_c * (math.dot(J1, v1) + math.dot(J2, v2));

        ClampLambda(ref lambda_t, lambda_n, ref accum_t, ref accum_n);

        return (J1*lambda_t, J2*lambda_t);
    }
    
    private void ClampLambda(ref float lambda_t, float lambda_n, ref float accum_t, ref float accum_n) {
        if (CollisionSystem.accumulateImpulses) {
            float old_tAccumulated = accum_t;
            accum_t = math.clamp(accum_t + lambda_t, -accum_n*frictionCoeff, accum_n*frictionCoeff);
            lambda_t = accum_t - old_tAccumulated;
        } else {
            lambda_t = math.clamp(lambda_t, -lambda_n*frictionCoeff, lambda_n*frictionCoeff);
        }
    }
}

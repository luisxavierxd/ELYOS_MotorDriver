#pragma once
#include <math.h>

namespace elyos_foc {

    // Constantes matemáticas
    constexpr float SQRT3 = 1.73205080757f;
    constexpr float SQRT3_2 = 0.86602540378f;
    constexpr float ONE_BY_SQRT3 = 0.57735026919f;
    constexpr float TWO_THIRDS = 0.66666666666f;

    struct PhaseCurrents {
        float a;
        float b;
        float c;
    };

    struct DQCurrents {
        float d;
        float q;
    };

    struct AlphaBeta {
        float alpha;
        float beta;
    };

    struct DQVoltages {
        float d;
        float q;
    };

    struct PhaseVoltages {
        float a;
        float b;
        float c;
    };

    // Controlador PI genérico
    class PIController {
    public:
        PIController(float kp, float ki, float limit) 
            : kp_(kp), ki_(ki), limit_(limit), integral_(0.0f) {}

        float operator()(float error, float dt) {
            integral_ += ki_ * error * dt;
            
            // Anti-windup clamping
            if (integral_ > limit_) integral_ = limit_;
            if (integral_ < -limit_) integral_ = -limit_;

            float output = kp_ * error + integral_;
            
            // Saturation
            if (output > limit_) output = limit_;
            if (output < -limit_) output = -limit_;

            return output;
        }

        void reset() { integral_ = 0.0f; }

    private:
        float kp_, ki_, limit_, integral_;
    };

    // 1. Transformada de Clarke: 3 fases (A,B,C) -> 2 fases estáticas (Alpha, Beta)
    inline AlphaBeta clarke(const PhaseCurrents& current) {
        AlphaBeta ab;
        // Asumiendo sistema balanceado: Ia + Ib + Ic = 0
        ab.alpha = current.a;
        ab.beta = ONE_BY_SQRT3 * current.a + TWO_THIRDS * SQRT3 * current.b;
        return ab;
    }

    // 2. Transformada de Park: (Alpha, Beta) estáticas -> (D, Q) rotatorias
    inline DQCurrents park(const AlphaBeta& ab, float angle_rad) {
        float c = cosf(angle_rad);
        float s = sinf(angle_rad);
        DQCurrents dq;
        dq.d = ab.alpha * c + ab.beta * s;
        dq.q = ab.beta * c - ab.alpha * s;
        return dq;
    }

    // 3. Transformada Inversa de Park: (D, Q) rotatorias -> (Alpha, Beta) estáticas
    inline AlphaBeta inv_park(const DQVoltages& dq, float angle_rad) {
        float c = cosf(angle_rad);
        float s = sinf(angle_rad);
        AlphaBeta ab;
        ab.alpha = dq.d * c - dq.q * s;
        ab.beta = dq.d * s + dq.q * c;
        return ab;
    }

    // 4. SVPWM (Space Vector Pulse Width Modulation)
    // Toma V_alpha y V_beta y retorna ciclos de trabajo [0.0, 1.0] para las 3 fases
    // asumiendo limit_v como la tensión máxima disponible.
    PhaseVoltages svpwm(const AlphaBeta& v_ab, float v_bus);

} // namespace elyos_foc

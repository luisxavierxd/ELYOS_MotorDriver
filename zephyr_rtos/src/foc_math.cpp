#include "foc_math.h"

namespace elyos_foc {

    // Space Vector Modulation
    // Transforma V_alpha y V_beta en ciclos de trabajo para las fases (A, B, C)
    PhaseVoltages svpwm(const AlphaBeta& v_ab, float v_bus) {
        PhaseVoltages duties;
        
        // Evitar divisiones por cero
        if (v_bus <= 0.0f) {
            return {0.0f, 0.0f, 0.0f};
        }

        // Normalizamos el vector de voltaje por el voltaje del bus
        float u_alpha = v_ab.alpha / v_bus;
        float u_beta = v_ab.beta / v_bus;

        // Tiempos equivalentes de las fases usando modulación de punto medio (Center-Aligned)
        // Método de inyección de tercera armónica / SVPWM simplificado
        float tA = u_alpha;
        float tB = (-0.5f * u_alpha) + (SQRT3_2 * u_beta);
        float tC = (-0.5f * u_alpha) - (SQRT3_2 * u_beta);

        // Encontramos el máximo y el mínimo para centrar el PWM
        float tMax = tA;
        if (tB > tMax) tMax = tB;
        if (tC > tMax) tMax = tC;

        float tMin = tA;
        if (tB < tMin) tMin = tB;
        if (tC < tMin) tMin = tC;

        // Shift para centrar (inyección de armónica)
        float center_shift = -0.5f * (tMax + tMin);

        duties.a = tA + center_shift + 0.5f;
        duties.b = tB + center_shift + 0.5f;
        duties.c = tC + center_shift + 0.5f;

        // Saturación final (0.0 a 1.0)
        if (duties.a > 1.0f) duties.a = 1.0f; else if (duties.a < 0.0f) duties.a = 0.0f;
        if (duties.b > 1.0f) duties.b = 1.0f; else if (duties.b < 0.0f) duties.b = 0.0f;
        if (duties.c > 1.0f) duties.c = 1.0f; else if (duties.c < 0.0f) duties.c = 0.0f;

        return duties;
    }

} // namespace elyos_foc

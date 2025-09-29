// PID.cpp
#include "PID.h"

PID::PID(float kp, float ki, float kd, float min_output, float max_output) {
    // Inicializa os ganhos e limites com os valores fornecidos
    setGains(kp, ki, kd);
    setOutputLimits(min_output, max_output);

    // Por padrão, o limite do integrador é igual ao da saída
    setIntegralLimits(min_output, max_output);

    // Inicializa o filtro da derivada com um valor padrão (desabilitado)
    _tau = 0.0;
}

void PID::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID::setOutputLimits(float min, float max) {
    _min_output = min;
    _max_output = max;
}

void PID::setIntegralLimits(float min, float max) {
    _min_integral = min;
    _max_integral = max;
}

void PID::setDerivativeFilter(float tau) {
    // Tau é a constante de tempo do filtro. Um valor pequeno significa menos filtragem.
    // Um valor de 0 desabilita o filtro.
    if (tau >= 0) {
        _tau = tau;
    }
}

float PID::calculate(float setpoint, float pv, float dt) {
    if (dt <= 0) return _min_output; // Evita divisão por zero

    // --- 1. Cálculo do Erro ---
    float error = setpoint - pv;

    // --- 2. Termo Proporcional ---
    float proportional = _kp * error;

    // --- 3. Termo Integral com Anti-Windup ---
    _integral += _ki * error * dt;
    // Limita (clamp) o valor do integrador para evitar o "windup"
    _integral = constrain(_integral, _min_integral, _max_integral);

    // --- 4. Termo Derivativo com Filtro Passa-Baixas ---
    float derivative_raw = (error - _previous_error) / dt;
    // Aplicando o filtro: y[n] = (1-a)*y[n-1] + a*x[n]
    // Onde 'a' está relacionado com a constante de tempo 'tau'
    float alpha = dt / (_tau + dt);
    float derivative = (1 - alpha) * _prev_derivative + alpha * derivative_raw;
    
    // --- 5. Cálculo da Saída Final ---
    float output = proportional + _integral + (_kd * derivative);

    // --- 6. Saturação da Saída ---
    // Garante que a saída final esteja dentro dos limites definidos
    output = constrain(output, _min_output, _max_output);

    // --- 7. Atualização dos Estados para a Próxima Iteração ---
    _previous_error = error;
    _prev_derivative = derivative;

    return output;
}

void PID::reset() {
    _integral = 0.0;
    _previous_error = 0.0;
    _prev_derivative = 0.0;
}
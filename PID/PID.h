// PID.h
#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
public:
    // Construtor: Inicializa o controlador com os parâmetros essenciais
    PID(float kp, float ki, float kd, float min_output, float max_output);

    // Método principal: Calcula a saída do controlador
    float calculate(float setpoint, float pv, float dt);

    // Métodos para alterar os parâmetros em tempo de execução
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void setIntegralLimits(float min, float max);
    void setDerivativeFilter(float tau); // tau: constante de tempo do filtro

    // Reseta os estados internos do controlador
    void reset();

private:
    // Ganhos do controlador
    float _kp;
    float _ki;
    float _kd;

    // Limites da saída (para saturação)
    float _min_output;
    float _max_output;

    // Limites do integrador (Anti-windup)
    float _min_integral;
    float _max_integral;

    // Filtro da derivada
    float _tau; // Constante de tempo para o filtro passa-baixas
    float _prev_derivative = 0.0;

    // Estados internos do controlador
    float _integral = 0.0;
    float _previous_error = 0.0;
};

#endif // PID_H
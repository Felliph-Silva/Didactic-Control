#include <Modbus.h>
#include <ModbusSerial.h>

// Pinos
#define ENCA 2
#define ENCB 3
#define PWM 4
#define IN1 5
#define IN2 6

// Modbus
ModbusSerial mb;

// Registradores Modbus (endereços)
enum {
  REG_KP,              // 0 - kp (leitura/escrita)
  REG_KI,              // 1 - ki (leitura/escrita)
  REG_KD,              // 2 - kd (leitura/escrita)
  REG_VT,              // 3 - Referência filtrada (leitura)
  REG_V1FILT,          // 4 - Velocidade filtrada (leitura)
  REG_ERRO,            // 5 - Erro (leitura)
  REG_ENABLE           // 6 - 0 = desligado, 1 = ligado (leitura/escrita)
};

// Variáveis de controle
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;
long prevT = 0;
int posPrev = 0;

float v1Filt = 0;
float v1Prev = 0;

float eintegral = 0;
float eprev = 0;
float e = 0;

float kp = 0.32;
float ki = 0.8;
float kd = 0.2;

float vt = 0;
float vtFiltered = 0;
int pwr = 0;

float e_integral_max = 600;

// Funções principais
void setup() {
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  // Iniciar comunicação Modbus
  mb.config(&Serial, 19200, SERIAL_8N1);
  mb.setSlaveId(1);

  // Adicionar registradores de Holding
  mb.addHreg(REG_KP, kp * 100);
  mb.addHreg(REG_KI, ki * 100);
  mb.addHreg(REG_KD, kd * 100);
  mb.addHreg(REG_VT, 0);
  mb.addHreg(REG_V1FILT, 0);
  mb.addHreg(REG_ERRO, 0);
  mb.addHreg(REG_ENABLE, 0);
}

void loop() {
  mb.task();

  // Atualizar valores dos ganhos via SCADA
  kp = mb.Hreg(REG_KP) / 100.0;
  ki = mb.Hreg(REG_KI) / 100.0;
  kd = mb.Hreg(REG_KD) / 100.0;

  bool enable = mb.Hreg(REG_ENABLE);

  int pos;
  float velocity2;
  getEncoderData(pos, velocity2);

  float deltaT = 0;
  float v1 = computeVelocity(pos, deltaT);
  filterVelocities(v1);

  vt = mb.Hreg(REG_VT);

  if (enable) {
    float u = computePID(vt, v1Filt, deltaT);
    controlMotor(u);
  } else {
    setMotor(0, 0, PWM, IN1, IN2); // motor desligado
  }

  // Atualizar registradores para SCADA
  mb.Hreg(REG_VT, (int)(vt));
  mb.Hreg(REG_V1FILT, (int)(v1Filt * 100));
  mb.Hreg(REG_ERRO, (int)(e * 100));

  delay(50);
}

// Cálculo da velocidade
float computeVelocity(int pos, float &deltaT) {
  long currT = micros();
  deltaT = ((float)(currT - prevT)) / 1.0e6;
  float velocity = (pos - posPrev) / deltaT;
  posPrev = pos;
  prevT = currT;
  return velocity / 600.0 * 60.0;
}

void filterVelocities(float v1) {
  v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Prev = v1;
}

float computePID(float vt1, float v1Filt, float deltaT) {
  e = vt1 - v1Filt;
  eintegral += e * deltaT;
  eintegral = constrain(eintegral, -e_integral_max, e_integral_max);
  float ederiv = (e - eprev) / deltaT;
  eprev = e;
  return kp * e + ki * eintegral + kd * ederiv;
}

void controlMotor(float u) {
  int dir = (u < 0) ? -1 : 1;
  pwr = abs((int)u);
  pwr = constrain(pwr, 0, 255);
  setMotor(dir, pwr, PWM, IN1, IN2);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  digitalWrite(in1, dir == 1);
  digitalWrite(in2, dir == -1);
}

void getEncoderData(int &pos, float &velocity2) {
  noInterrupts();
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts();
}

void readEncoder() {
  int b = digitalRead(ENCB);
  int increment = (b > 0) ? 1 : -1;
  pos_i += increment;
  long currT = micros();
  float deltaT = ((float)(currT - prevT_i)) / 1.0e6;
  velocity_i = increment / deltaT;
  prevT_i = currT;
}

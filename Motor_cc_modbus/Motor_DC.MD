# Controle PID com Arduino via Modbus RTU

Este projeto implementa um sistema de controle PID de velocidade de motor DC com **leitura via encoder rotativo hall**, controle de potência por **PWM**, e comunicação **Modbus RTU** com supervisório SCADA (como o LAquis SCADA). ([Código de implementação](Motor_cc_modbus.ino)).

## Bibliotecas Utilizadas

* [Modbus-Arduino](https://github.com/andresarmento/modbus-arduino): Biblioteca usada para comunicação Modbus RTU.
* `Modbus.h` e `ModbusSerial.h` são os cabeçalhos principais que permitem ao Arduino atuar como *slave* na comunicação Modbus.

---

## Pinos Utilizados

| Pino | Função                           |
| ---- | -------------------------------- |
| 2    | Entrada A do encoder (ENCA)      |
| 3    | Entrada B do encoder (ENCB)      |
| 4    | Saída PWM para controle do motor |
| 5    | Direção do motor (IN1)           |
| 6    | Direção do motor (IN2)           |

---

## Funções e Estrutura do Código

### 1. **Leitura do Encoder**

* O encoder rotativo é lido via **interrupção** no pino `ENCA`.
* A direção é determinada com base no valor do pino `ENCB`.

```cpp
void readEncoder() {
  int b = digitalRead(ENCB);
  int increment = (b > 0) ? 1 : -1;
  pos_i += increment;
  ...
}
```

* A velocidade instantânea também é calculada no momento da interrupção com `micros()`.

### 2. **Cálculo da Velocidade**

* No `loop()`, a velocidade é recalculada com base na diferença de posição e tempo desde a última leitura:

```cpp
float computeVelocity(int pos, float &deltaT) {
  deltaT = (tempo atual - tempo anterior);
  velocity = (posição atual - posição anterior) / deltaT;
}
```

### 3. **Filtro de Velocidade**

* A velocidade é suavizada com um filtro digital do tipo **Butterworth** de segunda ordem:

```cpp
void filterVelocities(float v1) {
  v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
}
```

---

## Controlador PID

O PID é implementado com as seguintes equações:

```cpp
e = referência - velocidade_filtrada;
eintegral += e * deltaT;
ederiv = (e - eprev) / deltaT;
u = kp * e + ki * eintegral + kd * ederiv;
```

* Os ganhos `kp`, `ki` e `kd` são **ajustáveis via Modbus**.
* A integral do erro é limitada com `constrain()` para evitar **integral windup**.

---

## Comunicação Modbus RTU

A comunicação Modbus é feita via `Serial` (RS-232/USB). O Arduino atua como **slave ID 1**, com registradores `Holding Register` para troca de dados com o supervisório.

### Configuração:

```cpp
mb.config(&Serial, 19200, SERIAL_8N1);
mb.setSlaveId(1);
```

### Registradores usados:

| Endereço | Função                        | Tipo            |
| -------- | ----------------------------- | --------------- |
| 0        | `kp` (multiplicado por 100)   | leitura/escrita |
| 1        | `ki` (multiplicado por 100)   | leitura/escrita |
| 2        | `kd` (multiplicado por 100)   | leitura/escrita |
| 3        | `vt` - referência desejada    | leitura         |
| 4        | `v1Filt` - velocidade medida  | leitura         |
| 5        | `erro` - erro atual do PID    | leitura         |
| 6        | `enable` - liga/desliga motor | leitura/escrita |

Os ganhos são divididos por 100 no código para recuperar o valor real em ponto flutuante:

```cpp
kp = mb.Hreg(REG_KP) / 100.0;
```

---

## Controle do Motor

A função `controlMotor()` determina a **direção** e **potência** com base no valor do controle `u` (saída do PID):

```cpp
int dir = (u < 0) ? -1 : 1;
pwr = constrain(abs(u), 0, 255);
setMotor(dir, pwr, PWM, IN1, IN2);
```

A função `setMotor()` comanda os pinos de direção e PWM para movimentar o motor no sentido correto.

---

## Habilitação/Desabilitação

O sistema só atua sobre o motor se o registrador `REG_ENABLE` for igual a 1. Isso permite ligar/desligar o motor diretamente pelo supervisório.

---

## Exemplo de Integração com SCADA

Esse código pode ser usado com ferramentas como o **LAquis SCADA** para:

* Monitorar velocidade real e erro;
* Alterar em tempo real os ganhos do PID;
* Aplicar diferentes referências de velocidade (`vt`);
* Ligar/desligar o motor.

---

## Aplicações

* Controle de motores DC com feedback.
* Bancadas didáticas de automação e controle.
* Integração com sistemas SCADA industriais via Modbus.

---

## Requisitos de Hardware

* Arduino UNO, Nano ou Mega.
* Ponte H (ex: L298N).
* Motor DC com encoder rotativo hall.
* Fonte de alimentação adequada.
* Conversor USB-RS485 (se for usar RS485 físico).

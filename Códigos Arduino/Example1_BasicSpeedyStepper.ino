
// ******************************************************************
//
// Teste do movimento do Motor usando a Biblioteca SpeedyStepper 
// adaptado por Gustavo Murta 06/abr/2020
//
//  Documentation for this library can be found at:
//    https://github.com/Stan-Reifel/SpeedyStepper
//
//  Arduino Mega - Arduino IDE 1.8.10 
//  Driver de Motor de passo - Passo completo / corrente 1 A
//  Motor de passo - 180 pulsos/volta (180 PPR) Meu motor é especial ! 
//
//  Passos por volta = 180 SPR (steps/rev) normalmente é 200  
//  SPS  (steps/sec)  (velocidade passos/segundo) 
//  RPS (rev/sec) = SPS / SPR      exemplo RPS = 1800/180 = 10 RPS  (Voltas/Segundo)
//  RPM = 60 x RPS = 10 x 60 = 600 RPM 
//
// ***********************************************************************

#include <SpeedyStepper.h>

const int LED_PIN = 13;                             // Led do Arduino Mega 
const int MOTOR_STEP_PIN = 5;                       // pino do pulso para o driver do motor 
const int MOTOR_DIRECTION_PIN = 6;                  // pino da direção para o driver do motor 
int SpeedInStepsPerSecond = 1800;                   // velocidade em passos/segundo 
int AccelerationInStepsPerSecondPerSecond = 3000;   // aceleracao em passos/segundo2 

SpeedyStepper stepper;                              // criação do objeto stepper 

void setup() 
{  
  pinMode(LED_PIN, OUTPUT);                                       // configura pino do Led como saída 
  Serial.begin(115200);                                           // console Serial 115200 bps 
  stepper.setStepsPerRevolution (180);                            // Passos por revolução ou por volta - o meu motor é especial 
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);     // configura os pinos do driver do motor 
}

void loop() 
{  
  stepper.setSpeedInStepsPerSecond(SpeedInStepsPerSecond);                                     // configura velocidade do motor 
  stepper.setAccelerationInStepsPerSecondPerSecond(AccelerationInStepsPerSecondPerSecond);     // configura aceleracao do motor
 
  stepper.moveRelativeInSteps(3600);                                                           // motor gira X passos em um sentido  
  delay(1000);                                                                                 // atraso de um segundo 
  
  stepper.moveRelativeInSteps(-3600);                                                          // gira X passos no sentido contrário 
  delay(1000);                                                                                 // atraso de um segundo
}


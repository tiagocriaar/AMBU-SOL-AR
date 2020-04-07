/*Projeto SAMBU Sol e Ar - versão 1.0  ( FRAGMENTOS DO PROGRAMA!!!) AGUARDEM 

  https://github.com/tiagocriaar/AMBU-SOL-AR

  Arduino Mega 2560  - Arduino IDE 1.8.10
  Motor de Passo NEMA23 15 kgf.cm / Driver de motor WD2404
  Display LCD 20x4 I2C / teclado de botoes

  Desenvolvido por: Gustavo Murta - 06/abr/2020

  https://github.com/mathertel/LiquidCrystal_PCF8574   Biblioteca do Display
  http://playground.arduino.cc/Code/Keypad             Biblioteca do teclado
  https://github.com/Stan-Reifel/SpeedyStepper         Biblioteca SpeddyStepper

*/

#include <Wire.h>                                  // Biblioteca Wire 
#include <LiquidCrystal_PCF8574.h>                 // Biblioteca para LCD com I2C
#include <Keypad.h>                                // Biblioteca para teclado 
#include <SpeedyStepper.h>                         // Biblioteca controle do motor de passo

LiquidCrystal_PCF8574 lcd(0x3F);                   // Endereço I2C - LCD PCF8574
const byte linhas = 2;                             // linhas display LCD
const byte colunas = 3;                            // colunas display LCD

char Keys[linhas][colunas] =                       // Definicao dos valores das botoes
{
  {'1', '3', '5'},
  {'2', '4', '6'}
};

byte LinhaPINO[linhas] = {13, 9};                  // Linhas do teclado LIN1 e LIN2
byte ColunaPINO[colunas] = {12, 11, 10};           // Colunas do teclado COL1,COL2 e COL3

Keypad keypad = Keypad( makeKeymap(Keys), LinhaPINO, ColunaPINO, linhas, colunas );
boolean blink = false;                             // variavel blink

int MOTOR_STEP_PIN = 5;                            // Porta digital D05 - pulso de passo no WD2404
int MOTOR_DIRECTION_PIN = 6;                       // Porta digital D06 - direção do motor no WD2404
int MOTOR_ENABLE_PIN = 7;                          // Porta digital D07 - ativa motor no WD2404
int StepsPerRevolution = 180;                      // passos por volta - meu motor é especial (mude para 200)
int directionTowardHome = -1;                      // sentido do motor = 1 para Ambu, -1 para home
float speedInStepsPerSecond;                       // velocidade em passos/segundo
bool limitSwitchFlag;

SpeedyStepper stepper;                             // cria objeto stepper

int swHomePosition = 19;                           // switch Home position = D19
int swAmbu = 18;                                   // switch mimitador no Ambu = D18

int pino22 = 22;                                   // pino de teste D22
int pino23 = 23;                                   // pino de teste D23

int SW1 = 0;                                       // dip switch SW1 WD2404 - selecao modo de passo
int SW2 = 0;                                       // dip switch SW2 WD2404 - selecao modo de passo
int SW3 = 0;                                       // dip switch SW3 WD2404 - selecao modo de passo

String modoPasso ;                                 // Modo de Passo do Motor
int selModo = 5 ;                                  // Selecao Modo de Passo completo
int pulsoNegativo = 2500;                          // Periodo negativo do pulso em microsegundos
int pulsoPositivo = 40;                            // Periodo positivo do pulso em microsegundos
int Periodo = 0;                                   // Periodo corrigido do Pulso de passo
float PPS = 0;                                     // Pulsos por segundo
bool sentido = true;                               // Variavel de sentido horario ou anti-horario
long PPR = 0;                                      // Numero de pulsos por volta
long Pulsos;                                       // Pulsos para o driver do motor
long PulsosCont;                                   // Pulsos contados para o driver do motor
int Voltas = 4;                                    // voltas do motor
int selectPPS = 5;                                 // selecao PPS
float RPS;                                         // Rotacoes por segundo
float RPM;                                         // Rotacoes por minuto
bool stopMotor = false;                            // Movimento do motor

void setup()
{

  pinMode(MOTOR_ENABLE_PIN, OUTPUT);                              // Configura pino ENA WD2404 como saída
  pinMode(MOTOR_DIRECTION_PIN, OUTPUT);                           // Configura pino DIR WD2404 como saída
  pinMode(MOTOR_STEP_PIN, OUTPUT);                                // Configura pino PUL WD2404 como saída
  pinMode(swHomePosition, INPUT);                                 // Configura pino swHomePosition como entrada
  pinMode(swAmbu, INPUT);                                         // Configura pino swAmbu como entrada
  pinMode(pino22, OUTPUT);                                        // Configura pino22 como saída
  pinMode(pino23, OUTPUT);                                        // Configura pino23 como saída

  stepper.setStepsPerRevolution (StepsPerRevolution);             // configura passos por volta - o meu motor é especial
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);     // configura pinos no driver WD2404

  int error;

  Wire.begin();                                    // Inicializa interface do LCD
  Wire.beginTransmission(0x3F);                    // incializa interface I2C
  error = Wire.endTransmission();                  // termina transmissão se houver erro

  lcd.begin(20, 4);                                // inicializa LCD 20x4
  Inicia_LCD ();                                   // LCD incial
  LCD_HomePosition ();                             // Print Home Position no LCD
  setHomePosition();                               // posiciona a aba na Home Position
  
  keypad.addEventListener(keypadEvent);            // evento para leitura do teclado

  attachInterrupt(digitalPinToInterrupt(19), int4, FALLING);    // Interrupção INT4(D19)= switch Home position
  attachInterrupt(digitalPinToInterrupt(18), int5, FALLING);    // Interrupção INT5(D18)= switch Ambu
}

// ------------------------- Funções do Display LCD ----------------------

void Inicia_LCD ()                           // Tela inicial no display LCD
{
  lcd.setBacklight(255);                     // apaga backlight
  lcd.home(); lcd.clear();                   // limpa display LCD
  lcd.setCursor(2, 0);                       // coluna 0 e linha 0
  lcd.print("AMBU Sol e AR");                // mostra no LCD
  lcd.setCursor(2, 1);                       // coluna 1 e linha 1
  lcd.print("RESPIRADOR");                   // mostra no LCD
  lcd.setCursor(2, 2);                       // coluna 1 e linha 1
  lcd.print("Versao 1.0");                   // mostra no LCD
  lcd.setCursor(2, 3);                       // coluna 0 e linha 2
  lcd.print("ABRIL - 2020");                 // mostra no LCD
  delay (2000);                              // atraso 2 segundos
}

void LCD_HomePosition ()                     // Print Home Position no LCD
{
  lcd.setBacklight(255);                     // luz de fundo LCD
  lcd.home(); lcd.clear();                   // limpa tela LCD
  lcd.setCursor(2, 0);                       // coluna 0 e linha 0
  lcd.print("AMBU Sol e AR");                // mostra no LCD
  lcd.setCursor(2, 2);                       // coluna 0 e linha 2
  lcd.print("Home Position");                // mostra no LCD
  delay (500);                               // atraso 0,5 segundos
}
 
///////////////////////////////////////////////////////////////////////////


// ------------------------- LCD Switches -----------------------------------

void LCD_swHomePosition ()                   // mostra switch Home Position
{
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("SW Home Position = ON ");       // mostra no LCD
}

void LCD_swAmbu ()                           // mostra switch Ambu
{
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("SW Ambu = ON ");                // mostra no LCD
}




// ------------------------- Funções do Teclado ----------------------

void keypadEvent(KeypadEvent key)
{
  switch (keypad.getState())                  // verifica se botão pressionado
  {
    case PRESSED:                             // identifica o botão pressionado

      if ((key == '1') && (digitalRead(swHomePosition) == HIGH))  // Botão 1 para Subir Eixo Z
      {
        LCD_pressionaAmbu ();                 // mostra Sobe Eixo Z
        pressionaAmbu ();                     // Sobe Eixo Z
      }

      if ((key == '2') && (digitalRead(swAmbu) == HIGH))  // Botao 2 para Descer Eixo Z
      {
        LCD_soltaAmbu ();                    // LCD desce Eixo Z
        soltaAmbu ();                        // Desce Eixo Z
      }

      if (key == '3')                         // Botao 3  para selecionar PPS
      {
        
      }

      if (key == '4')                         // Botao 4 para numero de voltas
      {
        
      }

      if (key == '5')                         // Botao 5  sem função ainda
      {
      }

      if (key == '6')                         // Botao 6 para Zerar Encoder Rotativo
      {
      }
  }
}

// ------------------------- Controle do Motor de Passo ----------------------

void disableWD2404()
{
  digitalWrite(MOTOR_ENABLE_PIN, LOW);         // Desativa o driver WD2404
  delay (10);                                  // Atraso de 10 milisegundos
}

void enableWD2404()
{
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);        // Ativa o driver WD2404
  delay (10);                                  // Atraso de 10  milisegundos
}

void FULL()
{
  modoPasso = "FULL" ;                        // Passo Completo
  SW1 = 1; SW2 = 1; SW3 = 0;                  // dip switches WD2404
  PPR = StepsPerRevolution;                   // PPR pulsos por volta
}

void HALFA()
{
  modoPasso = "HALFA" ;                       // Meio Passo A
  SW1 = 1; SW2 = 0; SW3 = 1;                  // dip switches WD2404
  PPR = 2 * StepsPerRevolution;               // PPR pulsos por volta
}

void P1_4()
{
  modoPasso = "MP-1/4" ;                      // Micropasso 1/4
  SW1 = 1; SW2 = 1; SW3 = 0;                  // dip switches WD2404
  PPR = 800;                                  // PPR pulsos por volta
}

void P1_8()
{
  modoPasso = "MP-1/8" ;                      // Micropasso 1/8
  SW1 = 0; SW2 = 1; SW3 = 0;                  // dip switches WD2404
  PPR = 1600;                                 // PPR pulsos por volta
}

void P1_16()
{
  modoPasso = "MP-1/16" ;                     // Micropasso 1/16
  SW1 = 1; SW2 = 0; SW3 = 0;                  // dip switches WD2404
  PPR = 3200;                                 // PPR pulsos por volta
}

void HALFB()
{
  modoPasso = "HALFB" ;                       // Meio Passo B
  SW1 = 0; SW2 = 1; SW3 = 1;                  // dip switches WD2404
  PPR = 2 * StepsPerRevolution;               // PPR pulsos por volta
}

void setHomePosition()                                    // posiciona a aba na Home Position
{
  enableWD2404();                                         // ativa driver do motor
  speedInStepsPerSecond = 360;                            // velocidade do motor em passos/segundo
  stepper.setAccelerationInStepsPerSecondPerSecond(360);  // configura aceleraçao do motor
  long maxDistanceToMoveInSteps = 900;                    // maxima distancia percorrida em passos

  if (digitalRead(swHomePosition) == HIGH)                // se SW Home position não acionado
  {
    stepper.setSpeedInStepsPerSecond(speedInStepsPerSecond);                           // configura velocidade do motor em passos/segundo
    stepper.setupRelativeMoveInSteps(maxDistanceToMoveInSteps * directionTowardHome);  // movimenta motor no sentido para home
    limitSwitchFlag = false;
    while (!stepper.processMovement())                       // enquanto estiver girando o motor
    {
      if (digitalRead(swHomePosition) == LOW)                // se o SW Home position for acionado
      {
        limitSwitchFlag = true;
        lcd.setCursor(2, 3);                                 // LCD coluna 0 e linha 0
        lcd.print("Posicao Home OK");                        // mostra no LCD
        break;                                               // para de girar motor
      }
    }
    if (limitSwitchFlag == false)                            // se SW Home position nunca for acionado
    {
      lcd.setCursor(2, 3);                                   // LCD coluna 0 e linha 0
      lcd.print("Home not OK");                              // mostra no LCD
    }
  }
  delay(25);                                                // atraso 25 ms

  stepper.setCurrentPositionInSteps(0L);                    // encontrou home - zera a posição em passos
}

////////////////////////////////////////////////////////////////////////

// ------------------------- Interrupções dos sensores ----------------------

void int4()                                   // Interrupção INT4(D19)= switch Home position
{
  PulsoPino22();                              // Pulso de teste pino D22
}

void int5()                                   // Interrupção INT5(D18)= switch Ambu
{
  PulsoPino23();                              // Pulso de teste pino D23
}


void PulsoPino22 ()                           // Pulso de teste pino D22
{
  digitalWrite(pino22, LOW);                  // pino 22 Low
  digitalWrite(pino22, HIGH);                 // pino 22 High
  delayMicroseconds(47);                      // atraso 50 microsegundos
  digitalWrite(pino22, LOW);                  // pino 22 low
}

void PulsoPino23 ()                           // Pulso de teste pino D23
{
  digitalWrite(pino23, LOW);                  // pino 23 Low
  digitalWrite(pino23, HIGH);                 // pino 23 High
  delayMicroseconds(47);                      // atraso 50 microsegundos
  digitalWrite(pino23, LOW);                  // pino 23 low
}

void loop()
{
  char key = keypad.getKey();                 // faz varredura dos botões
}

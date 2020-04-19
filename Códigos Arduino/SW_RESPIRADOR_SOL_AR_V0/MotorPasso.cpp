#include "MotorPasso.h"

//--------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------- Controle do Motor de Passo ------------------------------------------------------------------
void desativa_Driver_Motor() {
  digitalWrite(enable_MP, HIGH);           // Desativa o driver WD2404 = HIGH
  delay (10);                             // Atraso de 10 milisegundos
}

//--------------------------------------------------------------------------------------------------------------------------------------
void ativa_Driver_Motor() {
  digitalWrite(enable_MP, LOW);         // Ativa o driver WD2404 = LOW
  delay (10);                           // Atraso de 10  milisegundos
}

//--------------------------------------------------------------------------------------------------------------------------------------
void procedimento_avanca_MP() {
  digitalWrite(dir_MP, HIGH);            // Define sentido de giro para avançar fuso
  Serial.println("Fuso avançando");      // Mensagem no monitor: "Fuso avançando"
}

//-------------------------------------------------------------------------------------------------------------------------------------
void procedimento_recua_MP() {
  digitalWrite(dir_MP, LOW);             // Define sentido de giro para recuar fuso
  frequencia_MP = frequencia_MP;
  Serial.println("Fuso recuando");       // Mensagem no monitor: "Fuso recuando"
}

//-------------------------------------------------------------------------------------------------------------------------------------
void procedimento_ligar()
{
  digitalWrite(enable_MP, LOW);          // Habilita driver
  Serial.println("Motor Ligado");        // Mensagem no monitor: "Motor ligado"
}

//-------------------------------------------------------------------------------------------------------------------------------------
void procedimento_parar()
{
  digitalWrite(enable_MP, HIGH);         // Desabilita driver
  Serial.println("Motor Parado");        // Mensagem no monitor: "Motor parado"
}

void motorPressionaAmbu()                                                                   // movimenta a aba de pressão do Ambu
{

  ativa_Driver_Motor();                                                                     // ativa driver do motor
    
  stepper.setCurrentPositionInMillimeters (0);                                              // zera a posição em milimetros
  bool stopFlag = false;

  stepper.setSpeedInMillimetersPerSecond (velocidade_mm_por_seg);                           // configura velocidade do motor em mm/segundo
  stepper.setAccelerationInMillimetersPerSecondPerSecond(velocidade_mm_por_seg * 2);        // configura aceleraçao do motor - mm/seg 2
  stepper.setTargetPositionInMillimeters (percurso_Ambu * sentidoAmbu );                    // maxima distancia percorrida em milimetros = 100 (4 voltas)

  if ((digitalRead(CFC_Fim) == HIGH) && (sentidoAmbu == 1))                                 // se chave CFC_Fim não foi acionada
  {
    while (!stepper.motionComplete())                                                       // enquanto o motor não avançar todo percurso
    {
      stepper.processMovement();                                                            // gira o motor

      /*if (stepper.getCurrentPositionInMillimeters() == 100)                           // se a posição precorrida for igual a 90 mm
        {
        stepper.setSpeedInMillimetersPerSecond (velocidade_mm_por_seg/2);               // diminua a  velocidade do motor em mm/segundo
        }*/

      if ((digitalRead(CFC_Fim) == LOW) && (stopFlag == false))                            // se a chave CFC_Fim for acionada
      {
        stepper.setTargetPositionToStop();                                                // para o motor
        //stepper.setCurrentPositionInMillimeters (0);                                    // zera a posição em mm
        stopFlag = true;                                                                  // altera o estado do fla
      }
    }
  }

  if ((digitalRead(CFC_Inicio) == HIGH) && (sentidoAmbu == -1))                           // se chave CFC Inicio não foi acionada
  {
    while (!stepper.motionComplete())                                                       // enquanto o motor não avançar todo percurso
    {
      stepper.processMovement();                                                            // gira o motor

      if ((digitalRead(CFC_Inicio) == LOW) && (stopFlag == false))                            // se a chave CFC_Fim for acionada
      {
        stepper.setTargetPositionToStop();                                                // para o motor
        //stepper.setCurrentPositionInMillimeters (0);                                    // zera a posição em mm
        stopFlag = true;                                                                  // altera o estado do fla
      }
    }
  }
  desativa_Driver_Motor();
}

//--------------------------------------------------------------------------------------------------------------------------------------
// Faça testes iniciais do motor sem o mecanismo!

void inicializa_Ambu()                                                            // posiciona a aba na posição inicial
{
  LCD_inicializa_Respirador();                                                   // Print inicializa Ambu no LCD
  sentidoAmbu = -1;                                                               // sentido do motor = -1 para posição inicial
  percurso_Ambu = 100;                                                            // maxima distancia percorrida em milimetros
  ativa_Driver_Motor();                                                           // ativa driver do motor
  stepper.setCurrentPositionInMillimeters (0);                                    // zera a posição em milimetros
  bool limitSwitchFlag = false;                                                   // flag para switch de limite

  if (digitalRead(CFC_Inicio) == HIGH)                                            // se chave CFC_Inicio não foi acionada
  {
    stepper.setSpeedInMillimetersPerSecond (50);                                  // configura velocidade do motor em mm/segundo
    stepper.setAccelerationInMillimetersPerSecondPerSecond(200);                  // configura aceleraçao do motor - mm/s2
    stepper.setTargetPositionInMillimeters (percurso_Ambu * sentidoAmbu );        // maxima distancia percorrida em milimetros = 100 (4 voltas)

    while (!stepper.motionComplete())                                             // enquanto o motor não avançar todo percurso
    {
      stepper.processMovement();                                                  // gira o motor

      if (digitalRead(CFC_Inicio ) == LOW && (limitSwitchFlag == false))          // se a chave CFC_Inicio for acionada
      {
        stepper.setTargetPositionToStop();                                        // para o motor
        limitSwitchFlag = true;                                                   // muda o estado do flag
        LCD_posiciona(3, 3);                                                      // LCD coluna 3 e linha 3
        lcd.print("CFC Inicio OK    ");                                           // mostra no LCD
        LCD_mostra();
      }
    }
    if ((stepper.motionComplete () == true) && (limitSwitchFlag == false))        // se a chave CFC_Inicio nunca for acionada
    {
      LCD_posiciona(1, 3);                                                        // LCD coluna 1 e linha 3
      lcd.print("CFC Inicio not OK");                                             // mostra no LCD
      LCD_mostra();
    }
  }
  else                                                                            // se a chave CFC_Inicio estiver acionada
  {
    stepper.setCurrentPositionInMillimeters (0);                                  // zera a posição em mm
    limitSwitchFlag = true;                                                       // muda o estado do flag
    LCD_posiciona(3, 3);                                                          // LCD coluna 3 e linha 3
    lcd.print("CFC Inicio OK    ");                                               // mostra no LCD
    LCD_mostra();
  }
  desativa_Driver_Motor();
}

#include "controle_ambu.h"
#include "LCD_functions.h"
#include "controle_MP.h"
//--------------------------------------------------------------------------------------------------------------------------------------
//  sentido do motor = -1 para Ambu(anti-horário), 1 para posição inicial (horário)
//  Comprimento do eixo sem fim = 200 mm
//  Curso para compressão do AMBU = 100 mm ou 130 mm
//  FUSO 25 mm - (25 mm de avanço por volta)
//  Driver WED2404 configurado para meio passo 2/A  = 400 passos/volta
//  Para avançar os 100 mm são necessários 4 voltas no motor ou 4 x 400 passos = 1600 passos
//  Para avançar os 130 mm são necessários 5,2 voltas no motor ou 5,2 x 400 passos = 2080 passos

//--------------------------------------------------------------------------------------------------------------------------------------
// Faça testes iniciais do motor sem o mecanismo!

void inicializa_Ambu()                                                            // posiciona a aba na posição inicial
{
  LCD_inicializa_Respirador();                                                   // mostra inicializa Ambu no LCD
  sentidoAmbu = 1;                                                                // sentido do motor = 1 para posição inicial
  percurso_Ambu = 100;                                                            // maxima distancia percorrida em milimetros
  stepper.setCurrentPositionInMillimeters (0);                                    // zera a posição em milimetros
  bool limitSwitchFlag = false;                                                   // flag para switch de limite
  ativa_Driver_Motor();                                                           // ativa driver do motor

  if (digitalRead(CFC_Inicio) == HIGH)                                            // se chave CFC_Inicio não foi acionada
  {
    stepper.setSpeedInMillimetersPerSecond (50);                                  // configura velocidade do motor em mm/segundo
    stepper.setAccelerationInMillimetersPerSecondPerSecond(150);                  // configura aceleraçao do motor - mm/s2
    stepper.setTargetPositionInMillimeters (percurso_Ambu * sentidoAmbu );        // maxima distancia percorrida em milimetros = 100 (4 voltas)

    while (!stepper.motionComplete())                                             // enquanto o motor não avançar todo percurso
    {
      stepper.processMovement();                                                  // gira o motor

      if (digitalRead(CFC_Inicio ) == LOW && (limitSwitchFlag == false))          // se a chave CFC_Inicio for acionada
      {
        stepper.setTargetPositionToStop();                                        // para o motor
        limitSwitchFlag = true;                                                   // muda o estado do flag
        lcd.setCursor(3, 3);                                                      // LCD coluna 3 e linha 3
        lcd.print("CFC Inicio OK    ");                                           // mostra no LCD
      }
    }
    if ((stepper.motionComplete () == true) && (limitSwitchFlag == false))        // se a chave CFC_Inicio nunca for acionada
    {
      lcd.setCursor(1, 3);                                                        // LCD coluna 1 e linha 3
      lcd.print("CFC Inicio not OK");                                             // mostra no LCD
    }
  }
  else                                                                            // se a chave CFC_Inicio estiver acionada
  {
    stepper.setCurrentPositionInMillimeters (0);                                  // zera a posição em mm
    limitSwitchFlag = true;                                                       // muda o estado do flag
    lcd.setCursor(3, 3);                                                          // LCD coluna 3 e linha 3
    lcd.print("CFC Inicio OK    ");                                               // mostra no LCD
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------
// Faça testes iniciais do motor sem o mecanismo!

void motorPressionaAmbu ()                                                                  // movimenta a aba de pressão do Ambu
{
  ativa_Driver_Motor();                                                                     // ativa driver do motor
  stepper.setCurrentPositionInMillimeters (0);                                              // zera a posição em milimetros
  bool stopFlag = false;

  stepper.setSpeedInMillimetersPerSecond (velocidade_mm_por_seg);                           // configura velocidade do motor em mm/segundo
  //stepper.setAccelerationInMillimetersPerSecondPerSecond(velocidade_mm_por_seg * 3);        // configura aceleraçao do motor - mm/seg 2
  stepper.setAccelerationInMillimetersPerSecondPerSecond (480);
  stepper.setTargetPositionInMillimeters (percurso_Ambu * sentidoAmbu );                    // maxima distancia percorrida em milimetros = 100 (4 voltas)

  if ((digitalRead(CFC_Fim) == HIGH) && (sentidoAmbu == -1))                                // se chave CFC_Fim não foi acionada
  {
    while (!stepper.motionComplete())                                                       // enquanto o motor não avançar todo percurso
    {
      stepper.processMovement();                                                            // gira o motor

      /*if (stepper.getCurrentPositionInMillimeters() == 100)                               // se a posição precorrida for igual a 90 mm
        {
        stepper.setSpeedInMillimetersPerSecond (velocidade_mm_por_seg/2);                   // diminua a  velocidade do motor em mm/segundo
        }*/

      if ((digitalRead(CFC_Fim) == LOW) && (stopFlag == false))                             // se a chave CFC_Fim for acionada
      {
        stepper.setTargetPositionToStop();                                                  // para o motor
        //stepper.setCurrentPositionInMillimeters (0);                                      // zera a posição em mm
        stopFlag = true;                                                                    // altera o estado do fla
      }
    }
  }

  if ((digitalRead(CFC_Inicio) == HIGH) && (sentidoAmbu == 1))                              // se chave CFC Inicio não foi acionada
  {
    while (!stepper.motionComplete())                                                       // enquanto o motor não avançar todo percurso
    {
      stepper.processMovement();                                                            // gira o motor

      if ((digitalRead(CFC_Inicio) == LOW) && (stopFlag == false))                          // se a chave CFC_Fim for acionada
      {
        stepper.setTargetPositionToStop();                                                  // para o motor
        //stepper.setCurrentPositionInMillimeters (0);                                      // zera a posição em mm
        stopFlag = true;                                                                    // altera o estado do fla
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------
void respiradorAmbu ()                                                    // simulação do movimento do Ambu - Tecla B
{
  percurso_Ambu = 100;                                                    // percurso do movimento da aba em mm
  for (int i = 0; i <= 3; i++)                                            // repete 4 vezes para teste
  {
    //leitura_pressao_Ambu ();                                            // leitura de pressão do Ambu
    sentidoAmbu = -1;                                                     // sentido do motor = -1 para Ambu (anti-horário)
    velocidade_mm_por_seg = 134;                                          // INSP = 1,0s
    motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu
    delay(10) ;
    //delay(tempo_plato * 1000) ;                                         // atraso do tempo plato em ms (ex:0,25 x 1000) menos ajuste fino
    sentidoAmbu = 1;                                                      // sentido do motor = 1 oposto ao Ambu (horario)
    velocidade_mm_por_seg = 41;                                           // EXP = 2,5 s
    motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu
    //leitura_Encoder_Rotativo ();                                          // faz a leitura do Encoder Rotativo
  }
  digitalWrite(dir_MP, HIGH);
}

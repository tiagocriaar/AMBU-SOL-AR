#include "controle_MP.h"
#include "controle_ambu.h"


//---------------------------------------------------------------------------------------------------------------------------------------
void calibracao_velocidade_motor ()                                     // Calibração de velocidade de INS - Tecla 8
{
  percurso_Ambu = 100;                                                  // percurso do movimento da aba em mm
  sentidoAmbu = -1;                                                     // sentido do motor = -1 para Ambu (anti-horário)
  velocidade_mm_por_seg = 180 ;                                   // INS = 0,9 seg
  motorPressionaAmbu();                                                // movimenta a aba de pressão do Ambu
  //delay(10) ;
  sentidoAmbu = 1;                                                      // sentido do motor = 1 oposto ao Ambu (horario)
  velocidade_mm_por_seg = 134;                                 // EXP = 1,0
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu

  sentidoAmbu = -1;                                                     // sentido do motor = -1 para Ambu (anti-horário)
  velocidade_mm_por_seg = 134 ;                                   // INS = 1,0 seg
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu
  //delay(10) ;
  sentidoAmbu = 1;                                                      // sentido do motor = 1 oposto ao Ambu (horario)
  velocidade_mm_por_seg = 73;                                  // EXP = 1,5
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu

  sentidoAmbu = -1;                                                     // sentido do motor = -1 para Ambu (anti-horário)
  velocidade_mm_por_seg = 112 ;                                  // INS = 1,1 seg
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu
  //delay(10) ;
  sentidoAmbu = 1;                                                      // sentido do motor = 1 oposto ao Ambu (horario)
  velocidade_mm_por_seg = 52;                                  // EXP = 2,0
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu

  sentidoAmbu = -1;                                                     // sentido do motor = -1 para Ambu (anti-horário)
  velocidade_mm_por_seg = 98 ;                                    // INS = 1,2 seg
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu
  //delay(10) ;
  sentidoAmbu = 1;                                                      // sentido do motor = 1 oposto ao Ambu (horario)
  velocidade_mm_por_seg = 41;                                  // EXP = 2,5
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu

  sentidoAmbu = -1;                                                     // sentido do motor = -1 para Ambu (anti-horário)
  velocidade_mm_por_seg = 88 ;                                     // INS = 1,3
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu
  //delay(10) ;
  sentidoAmbu = 1;                                                      // sentido do motor = 1 oposto ao Ambu (horario)
  velocidade_mm_por_seg = 34;                                  // EXP = 3,0
  motorPressionaAmbu ();                                                // movimenta a aba de pressão do Ambu

  digitalWrite(dir_MP, HIGH);
}

//--------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------- Controle do Motor de Passo ------------------------------------------------------------------
void desativa_Driver_Motor()
{
  digitalWrite(enable_MP, HIGH);          // Desativa o driver WD2404 = HIGH
  delay (10);                             // Atraso de 10 milisegundos
}

//--------------------------------------------------------------------------------------------------------------------------------------
void ativa_Driver_Motor()
{
  digitalWrite(enable_MP, LOW);          // Ativa o driver WD2404 = LOW
  delay (10);                            // Atraso de 10  milisegundos
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

#include "MotorPasso.h"

//--------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------- Controle do Motor de Passo ------------------------------------------------------------------
void desativa_Driver_Motor()
{
  digitalWrite(enable_MP, LOW);           // Desativa o driver WD2404 = HIGH
  delay (10);                             // Atraso de 10 milisegundos
}

//--------------------------------------------------------------------------------------------------------------------------------------
void ativa_Driver_Motor()
{
  digitalWrite(enable_MP, HIGH);         // Ativa o driver WD2404 = LOW
  delay (10);                            // Atraso de 10  milisegundos
}

//--------------------------------------------------------------------------------------------------------------------------------------
void procedimento_avanca_MP()
{
  digitalWrite(dir_MP, HIGH);            // Define sentido de giro para avançar fuso
  Serial.println("Fuso avançando");      // Mensagem no monitor: "Fuso avançando"
}

//-------------------------------------------------------------------------------------------------------------------------------------
void procedimento_recua_MP()
{
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

#ifndef _MOTOR_PASSO_H_
#define _MOTOR_PASSO_H_

//--------------------------------------------------------------------------------------------------------------------------------------
//  Comprimento do eixo sem fim = 200 mm
//  Curso para compressão do AMBU = 100 mm ou 130 mm
//  FUSO 25 mm - (25 mm de avanço por volta)
//  Driver WED2404 configurado para 1/2 passo = 400 passos/volta
//  Para avançar os 100 mm são necessários 4 voltas no motor ou 4 x 400 passos = 1600 passos
//  Para avançar os 130 mm são necessários 5,2 voltas no motor ou 5,2 x 400 passos = 2080 passos

#include <FlexyStepper.h>                // Biblioteca controle do motor de passo
#include "config.h"
#include "IHM.h"

extern FlexyStepper stepper;
extern int frequencia_MP;
extern float velocidade_mm_por_seg;
extern float percurso_Ambu;
extern int sentidoAmbu;

void motorPressionaAmbu();                        // movimenta a aba de pressão do Ambu
void ativa_Driver_Motor();
void desativa_Driver_Motor();
void procedimento_ligar();                        // Liga o motor
void procedimento_parar();                        // Para o fuso
void procedimento_avanca_MP();                    // Avançar o fuso
void procedimento_recua_MP();                     // Recua o fuso
void inicializa_Ambu();                           // posiciona a aba na posição inicial

#endif

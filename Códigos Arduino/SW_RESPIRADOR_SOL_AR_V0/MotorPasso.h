#ifndef _MOTOR_PASSO_H_
#define _MOTOR_PASSO_H_

#include <FlexyStepper.h>                // Biblioteca controle do motor de passo
#include "config.h"

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

#endif

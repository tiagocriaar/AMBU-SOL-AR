#ifndef _MOTOR_PASSO_H_
#define _MOTOR_PASSO_H_

#include "config.h"

extern int frequencia_MP;

void ativa_Driver_Motor();
void desativa_Driver_Motor();
void procedimento_ligar();                        // Liga o motor
void procedimento_parar();                        // Para o fuso
void procedimento_avanca_MP();                    // Avançar o fuso
void procedimento_recua_MP();                     // Recua o fuso

#endif

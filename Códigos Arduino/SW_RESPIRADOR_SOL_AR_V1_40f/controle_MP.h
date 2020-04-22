#ifndef _CONTROLE_MP_H_
#define _CONTROLE_MP_H_
#include "Arduino.h"

extern int enable_MP;
extern int dir_MP;
extern float percurso_Ambu;
extern int sentidoAmbu;
extern float velocidade_mm_por_seg; 

void calibracao_velocidade_motor ();              // Calibração de velocidade de INS - Tecla D
void procedimento_ligar();                        // Liga o motor
void procedimento_parar();                        // Para o fuso
void desativa_Driver_Motor();
void ativa_Driver_Motor();
#endif

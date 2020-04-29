#ifndef _CONTROLE_AMBU_H
#define _CONTROLE_AMBU_H
#include <FlexyStepper.h>                // https://github.com/Stan-Reifel/FlexyStepper    

extern int sentidoAmbu;
extern float percurso_Ambu;
extern int pino_teste;
extern FlexyStepper stepper;
extern int velocidade_mm_por_seg;
extern int indice_velocidade;

extern int CFC_Inicio;
extern int CFC_Fim;

void inicializa_Ambu();
void motorPressionaAmbu ();
void respiradorAmbu ();

#endif

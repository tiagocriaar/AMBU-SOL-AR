#ifndef _MAIN_H_
#define _MAIN_H_

//--------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------
// Declaração das funções

void keypadEvent(KeypadEvent key, float Tempo_seg_Plato_OK, float Tempo_seg_Plato_NOK);
void procedimento_tag_efeito(int cod_efeito, char tag_efeito, char nome_efeito_LN2, char nome_efeito_LN3); // Lê o tipo de defeito
void procedimento_mostrar_causas_na_tela_do_micro(int cod_causa);                                          // mostrar causas na tela do micro

#endif

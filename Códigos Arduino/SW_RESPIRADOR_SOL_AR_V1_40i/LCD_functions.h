#ifndef _LCD_FUNCTIONS_H_
#define _LCD_FUNCTIONS_H_

#include <LiquidCrystal_PCF8574.h>       // https://github.com/mathertel/LiquidCrystal_PCF8574  
#include <RotaryEncoder.h>               // https://github.com/mathertel/RotaryEncoder  

extern RotaryEncoder encoder;
extern LiquidCrystal_PCF8574 lcd;

extern int newPos;
extern int lastPos1;
extern int lastPos2;
extern int lastPos3;
extern int lastPos4;

extern float valor_pressao_Ambu_inicial;

extern float frequencia_respiratoria;
extern float valor_pressao_Ambu;
extern int   tempo_inspiracao;
extern float tempo_inspiracao_seg;
extern float tempo_expiracao;
extern float divisor_relacao_insp_exp;
extern int   tempo_plato;
extern float tempo_plato_seg;
extern int   ponteiro_selecao;
extern int   contador_encoder;
extern float pausa_expiratoria;
extern int   ciclo_inspiratorio;
extern float ciclo_expiratorio;
extern float periodo_respiratorio;
extern int   indice_velocidade ;
extern int   velocidade_mm_por_seg; 

void configura_frequencia_respiratoria ();                                // configuração usando o rotary encoder
void configura_pressao_ambu ();                                           // configuração usando o rotary encoder
void configura_tempo_inspiracao ();                                       // configuração usando o rotary encoder
void configura_tempo_plato ();                                            // configuração usando o rotary encoder

void calculo_Parametros ();
void leitura_Encoder_Rotativo ();
void LCD_mostra_Parametros ();
void LCD_mostra_selecao_parametro ();
void LCD_Mostra_frequencia_respiratoria ();
void LCD_Mostra_Valor_Pressao_Ambu();                                      // Mostra valor do sensor de Pressão do Ambu
void LCD_inicializa_Respirador();                                          // Print inicializa Ambu no LCD
void LCD_TECLA_A_Pressionada(float Tempo_seg_Plato_OK);                    // Confirma tecla A pressionada para aceitar o valor do Tempo de Plato
void LCD_Mostra_Valor_Potenciometro_TP (float Tempo_seg_Plato_NOK);        // Mostra o valor do potenciômetro para o plato
void LCD_Mostra_causa(int cod_causa, char tag_causa, char nome_causa_LN2, char nome_causa_LN3);
void LCD_Mostra_Lista_Causas(int cod_causa, char tag_causa, char nome_causa_LN2, char nome_causa_LN3);
void LCD_Inicial();
void LCD_Teste_MP ();
void LCD_Teste_Potenciometro_TP ();                                        // Tela de teste do potenciometro do Tempo de Plato
void LCD_HomePosition ();
void LCD_iniciaMotor ();
void LCD_CFC_inicio ();
void LCD_CFC_fim ();
void LCD_avanca_Ambu();     // 1.40i
void LCD_retorna_Ambu();    // 1.40i
void LCD_Ligando_MP();
void LCD_Desligando_MP();

#endif

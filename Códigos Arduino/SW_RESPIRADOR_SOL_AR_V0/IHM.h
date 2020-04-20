#ifndef _IHM_H_
#define _IHM_H_
#include "config.h"
#if defined(IHM_RAMPS_128X64)
  #include <U8g2lib.h>
  extern U8G2_ST7920_128X64_F_SW_SPI lcd;
#else
  #include <LiquidCrystal_PCF8574.h>       // Biblioteca para LCD com I2C
  extern LiquidCrystal_PCF8574 lcd;
#endif

extern int frequencia_respiratoria;
extern float valor_pressao_Ambu;
extern float tempo_inspiracao;
extern float relacao_insp_exp;
extern float tempo_plato;
extern float valor_pot_TP;
extern float potenciometro_Plato;
extern float Tempo_seg_Plato_NOK;
extern float valor_pot_TP;
extern int ponteiro_selecao;

void LCD_mostra_selecao_parametro ();
void LCD_mostra();
void LCD_posiciona( byte c, byte l );
void procedimento_ler_pot_TP(float valor_pot_TP, float potenciometro_Plato, float Tempo_seg_Plato_NOK); //Lê a entrada do potenciômetro do Tempo de Plato
void procedimento_tag_causa(int cod_causa, char tag_causa, char nome_causa_LN2, char nome_causa_LN3); // Lê o tipo de causa
void LCD_mostra_Parametros();                             // Tela de configuração dos Parametros (favor não alterar)
void LCD_Mostra_Valor_Pressao_Ambu();        // Mostra valor do sensor de Pressão do Ambu
void LCD_inicializa_Respirador();                         // Print inicializa Ambu no LCD
void LCD_Inicial (String v);
void LCD_Teste_MP ();
void LCD_Teste_Potenciometro_TP ();     // Tela de teste do potenciometro do Tempo de Plato
void LCD_Mostra_Valor_Potenciometro_TP (float Tempo_seg_Plato_NOK); // Mostra o valor do potenciômetro para o plato
void LCD_TECLA_A_Pressionada(float Tempo_seg_Plato_OK); // Confirma tecla A pressionada para aceitar o valor do Tempo de Plato
void LCD_Mostra_causa(int cod_causa, String tag_causa, String nome_causa_LN2, String nome_causa_LN3);
void LCD_Mostra_Lista_Causas(int cod_causa, String tag_causa, String nome_causa_LN2, String nome_causa_LN3);
void LCD_HomePosition ();
void LCD_iniciaMotor ();
void LCD_CFC_inicio ();
void LCD_CFC_fim ();
void LCD_pressionaAmbu ();
void LCD_soltaAmbu ();
void LCD_Ligando_MP();
void LCD_Desligando_MP();

#endif

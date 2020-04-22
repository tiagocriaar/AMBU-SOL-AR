#include "LCD_functions.h"

//--------------------------------------------------------------------------------------------------------------------------------------
//--------------------------- Telas do Display LCD -------------------------------------------------------------------------------------

void LCD_Inicial ()    // Tela inicial no display LCD
{
  lcd.setBacklight(255);                                  // apaga backlight
  lcd.home(); lcd.clear();                                // limpa display LCD
  lcd.setCursor(0, 0);                                    // coluna 0 e linha 0
  lcd.print("*    RESPIRADOR    *");                      // mostra no LCD
  lcd.setCursor(0, 1);                                    // coluna 1 e linha 1
  lcd.print("*  AMBU  Sol e AR  *");                      // mostra no LCD
  lcd.setCursor(0, 2);                                    // coluna 1 e linha 2
  lcd.print("*   Versao 1.40f   *");                      // mostra no LCD
  lcd.setCursor(0, 3);                                    // coluna 0 e linha 3
  lcd.print("*   Abril - 2020   *");                      // mostra no LCD
  delay (500);                                            // atraso 0,5 segundos
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_inicializa_Respirador()                         // Print inicializa Ambu no LCD
{
  lcd.setBacklight(255);                                  // luz de fundo LCD
  lcd.home(); lcd.clear();                                // limpa tela LCD
  lcd.setCursor(0, 0);                                    // coluna 1 e linha 1
  lcd.print("*  AMBU  Sol e AR  *");                      // mostra no LCD
  lcd.setCursor(2, 2);                                    // coluna 0 e linha 2
  lcd.print("Inicializa AMBU");                           // mostra no LCD
  delay (500);                                            // atraso 0,5 segundos
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_mostra_Parametros ()                             // Tela de configuração dos Parametros (favor não alterar)
{
  lcd.setBacklight(255);                                  // luz de fundo LCD
  lcd.home(); lcd.clear();                                // limpa tela LCD
  lcd.setCursor(0, 0);                                    // coluna 0 e linha 0
  lcd.print("FREQ.RESP:");                                // mostra no LCD
  lcd.setCursor(11, 0);                                   // coluna 11 e linha 0
  lcd.print(frequencia_respiratoria, 1);                  // mostra no LCD
  lcd.setCursor(16, 0);                                   // coluna 0 e linha 0
  lcd.print("irpm");                                      // mostra no LCD
  lcd.setCursor(0, 1);                                    // coluna 0 e linha 1
  lcd.print("PRESSAO: ");                                 // mostra no LCD
  lcd.setCursor(10, 1);                                   // coluna 10 e linha 1
  lcd.print(valor_pressao_Ambu, 1);                       // mostra no LCD
  lcd.setCursor(15, 1);                                   // coluna 15 e linha 1
  lcd.print("cmH2O");                                     // mostra no LCD
  lcd.setCursor(0, 2);                                    // coluna 10 e linha 2
  lcd.print("INS:");                                      // mostra no LCD
  lcd.setCursor(5, 2);                                    // coluna 10 e linha 2
  lcd.print(tempo_inspiracao, 1);                         // mostra no LCD
  lcd.setCursor(8, 2);                                    // coluna 10 e linha 2
  lcd.print("s");                                         // mostra no LCD
  lcd.setCursor(10, 2);                                   // coluna 10 e linha 2
  lcd.print("I/E: 1:");                                   // mostra no LCD
  lcd.setCursor(17, 2);                                   // coluna 0 e linha 2
  lcd.print(relacao_insp_exp, 1);                         // mostra no LCD
  lcd.setCursor(0, 3);                                    // coluna 0 e linha 3
  lcd.print("Plato:");                                    // mostra no LCD
  lcd.setCursor(7, 3);                                    // coluna 0 e linha 3
  lcd.print(tempo_plato, 2);                              // mostra no LCD
  lcd.setCursor(11, 3);                                   // coluna 0 e linha 2
  lcd.print("s");                                         // mostra no LCD
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_mostra_selecao_parametro ()
{
  if (ponteiro_selecao == 1) {
    lcd.setCursor(10, 0); lcd.print(">");         // coluna 10 e linha 0
  }
  if (ponteiro_selecao == 2) {
    lcd.setCursor(10, 0); lcd.print(" ");         // coluna 10 e linha 0
    lcd.setCursor(9, 1); lcd.print(">");          // coluna 9 e linha 1
  }
  if (ponteiro_selecao == 3) {
    lcd.setCursor(9, 1); lcd.print(" ");          // coluna 9 e linha 1
    lcd.setCursor(4, 2); lcd.print(">");          // coluna 4 e linha 2
  }
  if (ponteiro_selecao == 4) {
    lcd.setCursor(4, 2); lcd.print(" ");          // coluna 4 e linha 2
    lcd.setCursor(14, 2); lcd.print(">");         // coluna 14 e linha 2
  }
  if (ponteiro_selecao == 5) {
    lcd.setCursor(14, 2); lcd.print(" ");         // coluna 14 e linha 2
    lcd.setCursor(6, 3); lcd.print(">");          // coluna 6 e linha 3
  }
  if (ponteiro_selecao == 0) {
    lcd.setCursor(6, 3); lcd.print(" ");          // coluna 6 e linha 3
  }
}

//-----------------------------------------------------------------------------------------------------------------------------------------
void LCD_Mostra_Valor_Pressao_Ambu ()        // Mostra valor do sensor de Pressão do Ambu
{
  lcd.setBacklight(255);                     // apaga backlight
  lcd.home(); lcd.clear();                   // limpa display LCD
  lcd.setCursor(3, 2);                       // coluna 3 e linha 2
  lcd.print("Pressao AMBU:");                // mostra no LCD
  lcd.setCursor(4, 3);                       // coluna 4 e linha 3
  lcd.print(valor_pressao_Ambu, 1);          // mostra no LCD
  lcd.setCursor(10, 3);                      // coluna 10 e linha 3
  lcd.print("cm H2O");                       // mostra no LCD
  delay(500);                                // atraso 0,5 seg
}



//  LCD DO SERGIO

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_CFC_inicio ()                       // mostra CFC_Início
{
  lcd.setCursor(0, 0);                       // coluna 0 e linha 0
  lcd.print("--------------------");         // mostra no LCD
  lcd.setCursor(0, 1);                       // coluna 0 e linha 1
  lcd.print("*                  *");         // mostra no LCD
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("* CFC Inicio = ON  *");         // mostra no LCD
  lcd.setCursor(0, 3);                       // coluna 0 e linha 3
  lcd.print("--------------------");         // mostra no LCD
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_CFC_fim ()                          // mostra CFC_Fim
{
  lcd.setCursor(0, 0);                       // coluna 0 e linha 0
  lcd.print("--------------------");         // mostra no LCD
  lcd.setCursor(0, 1);                       // coluna 0 e linha 1
  lcd.print("*                  *");         // mostra no LCD
  lcd.setCursor(0, 2);                       // coluna 0 e linha 2
  lcd.print("*   CFC Fim = ON   *");         // mostra no LCD
  lcd.setCursor(0, 3);                       // coluna 0 e linha 3
  lcd.print("--------------------");         // mostra no LCD
}

//---------------------------------------- Configuração dos Parâmetros ------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------
void configura_frequencia_respiratoria ()                                 // configuração usando o rotary encoder
{
  lcd.setCursor(10, 0);                                                   // coluna 9 e linha 0
  lcd.print(">");                                                         // mostra no LCD
  frequencia_respiratoria = 16 + newPos;                                  // frequencia_respiratoria inicial = 16
  if (frequencia_respiratoria < 10 )
  {
    if (frequencia_respiratoria < 5 ) frequencia_respiratoria = 5 ;       // frequencia minima = 5
    lcd.setCursor(11, 0);                                                 // coluna 11 e linha 0
    lcd.print("     ");                                                   // mostra no LCD
    lcd.setCursor(12, 0);                                                 // coluna 12 e linha 0
    lcd.print(frequencia_respiratoria, 1);                                // mostra no LCD
  }
  else
  {
    if (frequencia_respiratoria > 30) frequencia_respiratoria = 30 ;      // frequencia maxima 30 (acho que 60 não vai dar)
    lcd.setCursor(11, 0);                                                 // coluna 12 e linha 0
    lcd.print(frequencia_respiratoria, 1);                                // mostra no LCD
  }
  calculo_Parametros ();                                                  // efetua calculo dos parmetros
  delay (50);                                                             // atraso 50 ms
}

//---------------------------------------------------------------------------------------------------------------------------------------
void configura_pressao_ambu ()                                           // configuração usando o rotary encoder
{
  lcd.setCursor(9, 1);                                                   // coluna 9 e linha 1
  lcd.print(">");                                                        // mostra no LCD
  valor_pressao_Ambu  = 20 + newPos;                                     // pressão inicial = 20

  if (valor_pressao_Ambu  < 10 ) valor_pressao_Ambu = 10 ;               // pressão no Ambu minima = 10
  if (valor_pressao_Ambu > 60) valor_pressao_Ambu = 60 ;                 // pressão no Ambu maxima 60

  lcd.setCursor(10, 1);                                                  // coluna 10 e linha 1
  lcd.print(valor_pressao_Ambu, 1);                                      // mostra no LCD
  delay (50);                                                            // atraso 50 ms
}

//--------------------------------------------------------------------------------------------------------------------------------------
void configura_tempo_inspiracao ()                                       // configuração usando o rotary encoder
{
  lcd.setCursor(4, 2);                                                   // coluna 4 e linha 2
  lcd.print(">");                                                        // mostra no LCD
  tempo_inspiracao  = 1.0 + (newPos * 0.1) ;                             // tempo_inspiracao inicial = 1.0

  if (tempo_inspiracao  < 0.9 ) tempo_inspiracao = 0.9 ;                 // tempo_inspiracao minima = 0.9
  if (tempo_inspiracao  > 1.4 ) tempo_inspiracao = 1.4 ;                 // tempo_inspiracao maxima = 1.4

  lcd.setCursor(5, 2);                                                   // coluna 5 e linha 2
  lcd.print(tempo_inspiracao, 1);                                        // mostra no LCD
  calculo_Parametros ();                                                 // efetua calculo dos parmetros
  delay (50);                                                            // atraso 50 ms
}

//--------------------------------------------------------------------------------------------------------------------------------------
void configura_relacao_IE ()                                             // configuração usando o rotary encoder
{
  lcd.setCursor(14, 2);                                                  // coluna 14 e linha 2
  lcd.print(">");                                                        // mostra no LCD
  relacao_insp_exp  = 2.5 + (newPos * 0.5) ;                             // relacao_insp_exp inicial = 2.5

  if (relacao_insp_exp  < 1 ) relacao_insp_exp = 1 ;                     // relacao_insp_exp minima = 1
  if (relacao_insp_exp  > 4.5 ) relacao_insp_exp = 4.5 ;                 // relacao_insp_exp maxima = 4.5

  lcd.setCursor(17, 2);                                                  // coluna 17 e linha 2
  lcd.print(relacao_insp_exp, 1);                                        // mostra no LCD
  calculo_Parametros ();                                                 // efetua calculo dos parmetros
  delay (50);                                                            // atraso 50 ms
}

//--------------------------------------------------------------------------------------------------------------------------------------
void configura_tempo_plato ()                                           // configuração usando o rotary encoder
{
  lcd.setCursor(6, 3);                                                  // coluna 6 e linha 3
  lcd.print(">");                                                       // mostra no LCD
  tempo_plato  = 0.25 + (newPos * 0.05) ;                               // tempo_plato inicial = 0.25

  if (tempo_plato  < 0.1 ) tempo_plato = 0.1 ;                          // tempo_plato minima = 0.1
  if (tempo_plato  > 0.4 ) tempo_plato = 0.4 ;                          // tempo_plato maxima = 0.4

  lcd.setCursor(7, 3);                                                  // coluna 17 e linha 2
  lcd.print(tempo_plato, 2);                                            // mostra no LCD
  calculo_Parametros ();                                                // efetua calculo dos parmetros
  delay (50);                                                           // atraso 50 ms
}

//---------------------------------------- Cálculo dos Parâmetros ------------------------------------------------------------------
/*
   frequencia_respiratoria - 5 a 30 irpm (acho que não vai chegar no 60)
   valor_pressao_Ambu - 10 a 60 cmH2O
   tempo_inspiracao  - 0,9 a 1,4 seg
   relacao_insp_exp - 1 - 1,5 - 2,0 - 2,5 - 3,0 - 3,5 - 4,0 - 4,5
   tempo_plato - 0,1 - 0,15 - 0,2 - 0,25 - 0,3 - 0,35 - 0,40
*/
//----------------------------------------------------------------------------------------------------------------------------------
void calculo_Parametros ()
{
  pausa_expiratoria = 0.05;                                                               // intervalo medido entre EXP e INSP - 50 ms

  ciclo_inspiratorio = tempo_inspiracao + tempo_plato;                                    // calculo do ciclo inspiratorio em segundos
  ciclo_expiratorio = (relacao_insp_exp * tempo_inspiracao) + pausa_expiratoria;          // calculo do ciclo expiratorio em segundos

  periodo_respiratorio = ciclo_inspiratorio + ciclo_expiratorio;                          // calculo do período respiratório em segundos
  frequencia_respiratoria = 60 / periodo_respiratorio;                                    // calculo da frequencia respiratória em irpm

  lcd.setCursor(13, 3);                                                                   // coluna 6 e linha 3
  lcd.print("FR:");                                                                       // mostra no LCD
  lcd.setCursor(16, 3);                                                                   // coluna 17 e linha 2
  lcd.print(frequencia_respiratoria, 1);                                                  // mostra no LCD - frequencia calculada
}

//--------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------- Declaração das Funções ----------------------------------------------------------------------
void leitura_Encoder_Rotativo ()
{
  static int pos = 0;                     // lendo os contatos do  encoder
  encoder.tick();
  newPos = encoder.getPosition();

  if (pos != newPos)                      // se a posição foi alterada
  {
    if (ponteiro_selecao == 1) configura_frequencia_respiratoria ();
    if (ponteiro_selecao == 2) configura_pressao_ambu ();
    if (ponteiro_selecao == 3) configura_tempo_inspiracao ();
    if (ponteiro_selecao == 4) configura_relacao_IE ();
    if (ponteiro_selecao == 5) configura_tempo_plato ();
    pos = newPos;
  }
}

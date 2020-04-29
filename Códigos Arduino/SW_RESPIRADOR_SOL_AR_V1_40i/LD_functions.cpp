#include "LCD_functions.h"

//--------------------------------------------------------------------------------------------------------------------------------------
//--------------------------- Telas do Display LCD -------------------------------------------------------------------------------------

void LCD_Inicial ()    // Tela inicial no display LCD      // 1.40i
{
  lcd.setBacklight(255);                                  // apaga backlight
  lcd.home(); lcd.clear();                                // limpa display LCD
  lcd.setCursor(0, 0);                                    // coluna 0 e linha 0
  lcd.print("*    RESPIRADOR    *");                      // mostra no LCD
  lcd.setCursor(0, 1);                                    // coluna 1 e linha 1
  lcd.print("*  AMBU  Sol e AR  *");                      // mostra no LCD
  lcd.setCursor(0, 2);                                    // coluna 1 e linha 2
  lcd.print("*   Versao 1.40i   *");                      // mostra no LCD
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

//------------------------------------------------------------------------------------------------------------------------------------------1.40i
void LCD_avanca_Ambu()                                    // Print avança Ambu no LCD
{
  lcd.setBacklight(255);                                  // luz de fundo LCD
  lcd.home(); lcd.clear();                                // limpa tela LCD
  lcd.setCursor(0, 0);                                    // coluna 1 e linha 1
  lcd.print("*  AMBU  Sol e AR  *");                      // mostra no LCD
  lcd.setCursor(3, 2);                                    // coluna 0 e linha 2
  lcd.print("Avanca AMBU");                               // mostra no LCD
  lcd.setCursor(3, 3);                                    // coluna 3 e linha 3
  lcd.print("Veloc.=");                                   // mostra no LCD
  lcd.setCursor(11, 3);                                   // coluna 11 e linha 3
  lcd.print(velocidade_mm_por_seg);                       // mostra no LCD
  delay (50);                                             // atraso 50 ms
}

//------------------------------------------------------------------------------------------------------------------------------------------1.40i
void LCD_retorna_Ambu()                                   // Print avança Retorna no LCD
{
  lcd.setBacklight(255);                                  // luz de fundo LCD
  lcd.home(); lcd.clear();                                // limpa tela LCD
  lcd.setCursor(0, 0);                                    // coluna 1 e linha 1
  lcd.print("*  AMBU  Sol e AR  *");                      // mostra no LCD
  lcd.setCursor(3, 2);                                    // coluna 0 e linha 2
  lcd.print("Retorna AMBU");                              // mostra no LCD
  lcd.setCursor(3, 3);                                    // coluna 3 e linha 3
  lcd.print("Veloc.=");                                   // mostra no LCD
  lcd.setCursor(11, 3);                                   // coluna 11 e linha 3
  lcd.print(velocidade_mm_por_seg);                       // mostra no LCD
  delay (50);                                             // atraso 50 ms
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_mostra_Parametros ()                             // Tela de configuração dos Parametros (favor não alterar)
{
  lcd.setBacklight(255);                                  // luz de fundo LCD
  lcd.home(); lcd.clear();                                // limpa tela LCD
  lcd.setCursor(0, 0);                                    // coluna 0 e linha 0
  lcd.print("FR:");                                       // mostra no LCD
  LCD_Mostra_frequencia_respiratoria ();                  // mostra frequencia respiratoria
  lcd.setCursor(9, 0);                                    // coluna 0 e linha 0
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
  lcd.print(tempo_inspiracao_seg, 1);                     // mostra no LCD
  lcd.setCursor(8, 2);                                    // coluna 10 e linha 2
  lcd.print("s");                                         // mostra no LCD
  lcd.setCursor(10, 2);                                   // coluna 10 e linha 2
  lcd.print("I/E: 1:");                                   // mostra no LCD
  lcd.setCursor(17, 2);                                   // coluna 0 e linha 2
  lcd.print(divisor_relacao_insp_exp, 1);                 // mostra no LCD
  lcd.setCursor(0, 3);                                    // coluna 0 e linha 3
  lcd.print("Plato:");                                    // mostra no LCD
  lcd.setCursor(7, 3);                                    // coluna 0 e linha 3
  lcd.print(tempo_plato_seg, 2);                          // mostra no LCD
  lcd.setCursor(11, 3);                                   // coluna 0 e linha 2
  lcd.print("s");                                         // mostra no LCD
}

//------------------------------------------------------------------------------------------------------------------------------------------
void LCD_mostra_selecao_parametro ()
{
  if (ponteiro_selecao == 1) {
    lcd.setCursor(3, 0); lcd.print(">");         // coluna 3 e linha 0
    lcd.setCursor(9, 1); lcd.print(" ");         // coluna 9 e linha 1
    lcd.setCursor(4, 2); lcd.print(" ");         // coluna 4 e linha 2
    lcd.setCursor(6, 3); lcd.print(" ");         // coluna 6 e linha 3
  }
  if (ponteiro_selecao == 2) {
    lcd.setCursor(3, 0); lcd.print(" ");         // coluna 3 e linha 0
    lcd.setCursor(9, 1); lcd.print(">");         // coluna 9 e linha 1
    lcd.setCursor(4, 2); lcd.print(" ");         // coluna 4 e linha 2
    lcd.setCursor(6, 3); lcd.print(" ");         // coluna 6 e linha 3
  }
  if (ponteiro_selecao == 3) {
    lcd.setCursor(3, 0); lcd.print(" ");          // coluna 3 e linha 0
    lcd.setCursor(4, 2); lcd.print(">");          // coluna 4 e linha 2
    lcd.setCursor(9, 1); lcd.print(" ");          // coluna 9 e linha 1
    lcd.setCursor(6, 3); lcd.print(" ");          // coluna 6 e linha 3
  }
  if (ponteiro_selecao == 4) {
    lcd.setCursor(3, 0); lcd.print(" ");          // coluna 3 e linha 0
    lcd.setCursor(9, 1); lcd.print(" ");          // coluna 9 e linha 1
    lcd.setCursor(4, 2); lcd.print(" ");          // coluna 4 e linha 2
    lcd.setCursor(6, 3); lcd.print(">");          // coluna 6 e linha 3
  }
  if (ponteiro_selecao == 0) {
  }
}

//-----------------------------------------------------------------------------------------------------------------------------------------
void LCD_Mostra_frequencia_respiratoria ()                               // mostra frequencia respiratoria
{
  if (frequencia_respiratoria < 10 )
  {
    lcd.setCursor(4, 0);                                                  // coluna 11 e linha 0
    lcd.print("    ");                                                    // mostra no LCD
    lcd.setCursor(5, 0);                                                  // coluna 12 e linha 0
    lcd.print(frequencia_respiratoria, 1);                                // mostra no LCD
  }
  else
  {
    lcd.setCursor(4, 0);                                                  // coluna 12 e linha 0
    lcd.print(frequencia_respiratoria, 1);                                // mostra no LCD
  }
}

//-----------------------------------------------------------------------------------------------------------------------------------------
void LCD_Mostra_Valor_Pressao_Ambu ()        // mostra valor do sensor de Pressão do Ambu
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
void configura_frequencia_respiratoria ()                                // configuração da Frequencia Respiratoria
{
  frequencia_respiratoria = 16 + newPos;                                 // frequencia_respiratoria inicial = 16 / resolução 1
  lastPos1 = newPos;

  if (frequencia_respiratoria < 8 ) {
    encoder.setPosition(-8);                                             // limita a posição do encoder
    frequencia_respiratoria = 8 ;                                        // frequencia minima = 8
  }
  if (frequencia_respiratoria > 40) {
    encoder.setPosition(24);                                             // limita a posição do encoder
    frequencia_respiratoria = 40 ;                                       // frequencia maxima 40  (talvez o maximo seja 30)
  }

  calculo_Parametros ();                                                 // efetua calculo dos parmetros
  delay (50);                                                            // atraso 50 ms
}

//---------------------------------------------------------------------------------------------------------------------------------------
void configura_pressao_ambu ()                                           // configuração da Pressão do Ambu
{
  valor_pressao_Ambu  = 20 + newPos;                                     // pressão inicial = 20 cmH2O / resolução 1
  lastPos2 = newPos;

  if (valor_pressao_Ambu  < 15 ) {
    encoder.setPosition(-5);                                             // limita a posição do encoder
    valor_pressao_Ambu = 15 ;                                            // pressão no Ambu minima = 15
  }
  if (valor_pressao_Ambu > 40) {
    encoder.setPosition(20);                                             // limita a posição do encoder
    valor_pressao_Ambu = 40 ;                                            // pressão no Ambu maxima 40
  }

  lcd.setCursor(10, 1);                                                  // coluna 10 e linha 1
  lcd.print(valor_pressao_Ambu, 1);                                      // mostra no LCD
  delay (50);                                                            // atraso 50 ms
}

//--------------------------------------------------------------------------------------------------------------------------------------
void configura_tempo_inspiracao ()                                       // configuração do Tempo de Inspiração em milisegundos
{
  tempo_inspiracao  = 1000 + (newPos * 100) ;                            // tempo_inspiracao inicial = 1000 ms / resolução 100 ms
  lastPos3 = newPos;

  if (tempo_inspiracao  < 600 ) {
    encoder.setPosition(-4);                                             // limita a posição do encoder
    tempo_inspiracao = 600 ;                                             // tempo_inspiracao minima = 0.6 s
  }
  if (tempo_inspiracao  > 1800 ) {
    encoder.setPosition(8);                                              // limita a posição do encoder
    tempo_inspiracao = 1800 ;                                            // tempo_inspiracao maxima = 1.8 s
  }

  tempo_inspiracao_seg = (float(tempo_inspiracao)) / 1000;               // converte o tempo em segundos

  indice_velocidade = lastPos3 + 4 ;                                     // calcula o indice do vetor velocidade_inspiracao

  //lcd.setCursor(15, 3);                                                  // coluna 16 linha 3
  //lcd.print(indice_velocidade);                                          // mostra no LCD

  lcd.setCursor(5, 2);                                                   // coluna 5 e linha 2
  lcd.print(tempo_inspiracao_seg, 1);                                    // mostra no LCD em segundos
  calculo_Parametros ();                                                 // efetua calculo dos parametros
  delay (50);                                                            // atraso 50 ms
}


//--------------------------------------------------------------------------------------------------------------------------------------
void configura_tempo_plato ()                                           // configuração Tempo de Platô
{
  if (divisor_relacao_insp_exp >= 0 ) {
    tempo_plato  = 200 + (newPos * 50) ;                                // tempo_plato inicial = 200 ms / Resolução = 50 ms
    lastPos4 = newPos;
  }

  if (tempo_plato  < 0 ) {
    encoder.setPosition(-4);                                            // limita a posição do encoder
    tempo_plato = 0 ;                                                   // tempo_plato minima = 0
  }
  if (tempo_plato  > 500 ) {
    encoder.setPosition(6);                                             // limita a posição do encoder
    tempo_plato = 500 ;                                                 // tempo_plato maxima = 500 ms
  }

  tempo_plato_seg = (float(tempo_plato)) / 1000;                        // converte tempo em segundos

  lcd.setCursor(7, 3);                                                  // coluna 17 e linha 2
  lcd.print(tempo_plato_seg, 2);                                        // mostra no LCD
  calculo_Parametros ();                                                // efetua calculo dos parmetros
  delay (50);                                                           // atraso 50 ms
}

//---------------------------------------- Definição dos Parâmetros ------------------------------------------------------------------
/*
  Frequência Respiratória - 8 a 40 irpm (resolução 1)   Valor Padrão = 16 irpm
  Pressão no Ambu = 15 a 40 cmH2O (resolução 1)         Valor Padrão = 20 cmH2O
  Tempo de Inspiração = 0,6 a 1,8 s (resolução 0,1 s)   Valor Padrão = 1,0 s
  Relação I/E = tempo_inspiracao / tempo_expiracao      Valor Padrão = 1:2,5
  Tempo de Platô = 0 a 0,5 s (resolução 0,05 s)         Valor Padrão = 0,2 s
*/
//----------------------------------------------------------------------------------------------------------------------------------
void calculo_Parametros ()
{
  periodo_respiratorio = 60 / frequencia_respiratoria;                                    // calculo da período respiratório em segundos

  ciclo_inspiratorio = tempo_inspiracao + tempo_plato;                                    // calculo do ciclo inspiratorio em ms

  tempo_expiracao = (periodo_respiratorio * 1000)  - ciclo_inspiratorio;                  // calculo do tempo de expiração em ms

  divisor_relacao_insp_exp = tempo_expiracao / tempo_inspiracao;                          // calculo da relação I/E

  if (divisor_relacao_insp_exp >= 0)  {                                                   // se a relação IE for >= zero - válida
    lcd.setCursor(17, 2);                                                                 // coluna 17 e linha 2
    lcd.print(divisor_relacao_insp_exp, 1);                                               // mostra no LCD
  }
  else {
    lcd.setCursor(17, 2);                                                                 // coluna 17 e linha 2
    lcd.print("ERR");                                                                     // mostra no LCD
  }

  if (tempo_expiracao < 940)                                                              // se tempode expiração for < 950 ms
  {
    lcd.setCursor(16, 0);                                                                 // coluna 16 linha 0
    lcd.print("ERRO");                                                                    // mostra no LCD
  }
  else
  {
    lcd.setCursor(16, 0);                                                                 // coluna 16 linha 0
    lcd.print("    ");                                                                    // mostra no LCD
  }

  lcd.setCursor(16 , 3);                                                                  // coluna 16 linha 3
  lcd.print("    ");                                                                      // mostra no LCD
  lcd.setCursor(16 , 3);                                                                  // coluna 16 linha 3
  lcd.print(tempo_expiracao, 0);                                                          // mostra no LCD

  LCD_Mostra_frequencia_respiratoria ();                                                  // mostra frequencia respiratoria
}

//--------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------- Declaração das Funções ----------------------------------------------------------------------
void leitura_Encoder_Rotativo ()
{
  static int pos = 0;                       // lendo os contatos do  encoder
  encoder.tick();
  newPos = (encoder.getPosition() * 2 / 2); // para diminuir a sensibilidade

  if (pos != newPos)                        // se a posição foi alterada
  {
    if (ponteiro_selecao == 1) configura_frequencia_respiratoria ();
    if (ponteiro_selecao == 2) configura_pressao_ambu ();
    if (ponteiro_selecao == 3) configura_tempo_inspiracao ();
    if (ponteiro_selecao == 4) configura_tempo_plato ();
    pos = newPos;
  }
}
